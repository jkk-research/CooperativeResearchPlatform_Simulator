#include "simulator/scenario_data.hpp"

#include <matio.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace crp
{
namespace apl
{
namespace
{

using MatVariable = std::unique_ptr<matvar_t, decltype(&Mat_VarFree)>;

std::size_t count(const matvar_t * variable)
{
    if (!variable) {return 0;}
    std::size_t result = 1;
    for (int dimension = 0; dimension < variable->rank; ++dimension)
    {
        result *= variable->dims[dimension];
    }
    return result;
}

matvar_t * field(matvar_t * variable, const char * name, std::size_t index = 0)
{
    if (!variable || variable->class_type != MAT_C_STRUCT || index >= count(variable))
    {
        return nullptr;
    }
    return Mat_VarGetStructFieldByName(variable, name, index);
}

double number(const matvar_t * variable, std::size_t index)
{
    if (!variable || !variable->data || variable->isComplex || index >= count(variable))
    {
        throw std::runtime_error("Missing or invalid numeric MAT data");
    }
    double result;
    switch (variable->data_type)
    {
        case MAT_T_DOUBLE: result = static_cast<double *>(variable->data)[index]; break;
        case MAT_T_SINGLE: result = static_cast<float *>(variable->data)[index]; break;
        case MAT_T_INT8: result = static_cast<mat_int8_t *>(variable->data)[index]; break;
        case MAT_T_UINT8: result = static_cast<mat_uint8_t *>(variable->data)[index]; break;
        case MAT_T_INT16: result = static_cast<mat_int16_t *>(variable->data)[index]; break;
        case MAT_T_UINT16: result = static_cast<mat_uint16_t *>(variable->data)[index]; break;
        case MAT_T_INT32: result = static_cast<mat_int32_t *>(variable->data)[index]; break;
        case MAT_T_UINT32: result = static_cast<mat_uint32_t *>(variable->data)[index]; break;
        case MAT_T_INT64: result = static_cast<mat_int64_t *>(variable->data)[index]; break;
        case MAT_T_UINT64: result = static_cast<mat_uint64_t *>(variable->data)[index]; break;
        default: throw std::runtime_error("Unsupported MAT numeric type");
    }
    if (!std::isfinite(result)) {throw std::runtime_error("Non-finite MAT numeric value");}
    return result;
}

std::vector<double> series(matvar_t * variable, std::size_t length, double fallback)
{
    std::vector<double> result(length, fallback);
    if (count(variable) == 0) {return result;}
    for (std::size_t index = 0; index < length; ++index)
    {
        result[index] = number(variable, std::min(index, count(variable) - 1));
    }
    return result;
}

std::vector<MapPoint> points(matvar_t * variable)
{
    if (!variable || variable->rank != 2 || variable->dims[1] != 2)
    {
        throw std::runtime_error("Expected an Nx2 map-frame waypoint matrix");
    }
    std::vector<MapPoint> result;
    for (std::size_t index = 0; index < variable->dims[0]; ++index)
    {
        result.push_back({number(variable, index), number(variable, index + variable->dims[0])});
    }
    if (result.size() < 2) {throw std::runtime_error("A corridor needs at least two waypoints");}
    return result;
}

void probabilities(const std::vector<double> & values)
{
    for (double value : values)
    {
        if (value < 0.0 || value > 1.0)
        {
            throw std::runtime_error("Confidence must be in [0, 1]");
        }
    }
}

void loadActors(mat_t * file, const char * name, unsigned int classification,
    bool obstacle, ScenarioData & data, double horizon, bool trim_zero_padding)
{
    MatVariable variable(Mat_VarRead(file, name), Mat_VarFree);
    for (std::size_t actor_index = 0; actor_index < count(variable.get()); ++actor_index)
    {
        ScenarioActor actor;
        actor.classification = classification;
        auto label = field(variable.get(), "classification", actor_index);
        if (count(label) > 0)
        {
            const double value = number(label, 0);
            if (value < 0.0 || value > 11.0 || value != std::floor(value))
            {
                throw std::runtime_error("classification must be an Autoware label in [0, 11]");
            }
            actor.classification = static_cast<unsigned int>(value);
        }
        actor.obstacle = obstacle;
        auto position = field(variable.get(), "position", actor_index);
        auto position_x = field(position, "x");
        auto position_y = field(position, "y");
        const auto length = std::min({count(position_x), count(position_y), data.time.size()});
        if (length == 0) {throw std::runtime_error(std::string(name) + ": missing positions");}
        for (std::size_t index = 0; index < length; ++index)
        {
            actor.positions.push_back({number(position_x, index), number(position_y, index)});
        }
        auto velocity = field(variable.get(), "velocity", actor_index);
        if (velocity && velocity->class_type == MAT_C_STRUCT) {velocity = field(velocity, "velocity");}
        actor.speeds = series(velocity, length, 0.0);
        actor.existence_probabilities = series(
            field(variable.get(), "existence_probability", actor_index), length, 1.0);
        actor.main_confidences = series(field(variable.get(), "confidence", actor_index), length, 1.0);
        probabilities(actor.existence_probabilities);
        probabilities(actor.main_confidences);
        actor.dimensions = series(field(variable.get(), "size", actor_index), 3,
            classification == 7 ? 0.6 : 1.0);
        for (double dimension : actor.dimensions)
        {
            if (dimension <= 0.0) {throw std::runtime_error("Object dimensions must be positive");}
        }
        while (trim_zero_padding && !obstacle && actor.positions.size() > 1 &&
            actor.positions.back().x == 0.0 && actor.positions.back().y == 0.0 &&
            actor.speeds[actor.positions.size() - 1] == 0.0)
        {
            actor.positions.pop_back();
        }
        for (std::size_t index = 0; index < actor.positions.size(); ++index)
        {
            actor.headings.push_back(pathHeading(actor.positions, index));
        }
        actor.alternatives.resize(length);
        auto alternatives = field(variable.get(), "objectPossibleTrajectories", actor_index);
        for (std::size_t branch = 0; branch < count(alternatives); ++branch)
        {
            auto trajectory = field(alternatives, "trajectories", branch);
            auto trajectory_x = field(trajectory, "x");
            auto trajectory_y = field(trajectory, "y");
            auto confidence = field(alternatives, "confidence", branch);
            if (!trajectory_x || !trajectory_y || !confidence ||
                trajectory_x->rank != 2 || trajectory_y->rank != 2 || confidence->rank != 2)
            {
                throw std::runtime_error("Alternative trajectories need 2D x, y and confidence arrays");
            }
            const auto rows = std::min({trajectory_x->dims[0], trajectory_y->dims[0], confidence->dims[0]});
            const auto columns = std::min({trajectory_x->dims[1], trajectory_y->dims[1], confidence->dims[1], length});
            for (std::size_t observation = 0; observation < std::min(rows, columns); ++observation)
            {
                TrajectorySample sample;
                sample.confidence = number(confidence, observation + observation * confidence->dims[0]);
                probabilities({sample.confidence});
                if (sample.confidence == 0.0) {continue;}
                const auto end = std::min(rows, observation +
                    static_cast<std::size_t>(std::floor(horizon / data.time_step + 1e-8)) + 1);
                for (std::size_t future = observation; future < end; ++future)
                {
                    if (number(confidence, future + observation * confidence->dims[0]) <= 0.0) {break;}
                    sample.points.push_back({
                        number(trajectory_x, future + observation * trajectory_x->dims[0]),
                        number(trajectory_y, future + observation * trajectory_y->dims[0])});
                }
                if (!sample.points.empty()) {actor.alternatives[observation].push_back(std::move(sample));}
            }
        }
        data.actors.push_back(std::move(actor));
    }
}

}

double pathHeading(const std::vector<MapPoint> & points, std::size_t index)
{
    if (points.size() < 2) {return 0.0;}
    const auto origin = std::min(index, points.size() - 1);
    for (std::size_t next = origin + 1; next < points.size(); ++next)
    {
        const auto delta_x = points[next].x - points[origin].x;
        const auto delta_y = points[next].y - points[origin].y;
        if (std::hypot(delta_x, delta_y) > 1e-8) {return std::atan2(delta_y, delta_x);}
    }
    for (std::size_t previous = origin; previous > 0; --previous)
    {
        const auto delta_x = points[origin].x - points[previous - 1].x;
        const auto delta_y = points[origin].y - points[previous - 1].y;
        if (std::hypot(delta_x, delta_y) > 1e-8) {return std::atan2(delta_y, delta_x);}
    }
    return 0.0;
}

ScenarioData ScenarioData::load(const std::string & filename, double prediction_horizon,
    bool trim_zero_padding)
{
    if (!std::isfinite(prediction_horizon) || prediction_horizon <= 0.0)
    {
        throw std::runtime_error("Prediction horizon must be positive");
    }
    std::unique_ptr<mat_t, decltype(&Mat_Close)> file(Mat_Open(filename.c_str(), MAT_ACC_RDONLY), Mat_Close);
    if (!file) {throw std::runtime_error("Cannot open MAT scenario: " + filename);}
    ScenarioData data;
    MatVariable time(Mat_VarRead(file.get(), "time"), Mat_VarFree);
    if (count(time.get()) < 2) {throw std::runtime_error("Scenario needs at least two time samples");}
    data.time = series(time.get(), count(time.get()), 0.0);
    data.time_step = data.time[1] - data.time[0];
    if (std::abs(data.time[0]) > 1e-8 || data.time_step <= 0.0)
    {
        throw std::runtime_error("Scenario time must start at zero and increase");
    }
    for (std::size_t index = 1; index < data.time.size(); ++index)
    {
        if (std::abs(data.time[index] - data.time[index - 1] - data.time_step) > 1e-7)
        {
            throw std::runtime_error("Scenario time samples must be uniformly spaced");
        }
    }
    MatVariable target(Mat_VarRead(file.get(), "targetPath_qg"), Mat_VarFree);
    data.target_path = points(target.get());
    MatVariable speed(Mat_VarRead(file.get(), "speedLimit"), Mat_VarFree);
    if (count(speed.get()) != 1 && count(speed.get()) != data.target_path.size())
    {
        throw std::runtime_error("speedLimit must be scalar or match targetPath_qg");
    }
    data.speed_limits = series(speed.get(), data.target_path.size(), 0.0);
    for (double limit : data.speed_limits)
    {
        if (limit < 0.0) {throw std::runtime_error("Speed limits cannot be negative");}
    }
    MatVariable lanes(Mat_VarRead(file.get(), "lanes"), Mat_VarFree);
    if (!lanes || lanes->class_type != MAT_C_CELL || count(lanes.get()) == 0)
    {
        throw std::runtime_error("Scenario needs a lanes cell array to preserve corridor boundaries");
    }
    for (std::size_t index = 0; index < count(lanes.get()); ++index)
    {
        data.lanes.push_back(points(Mat_VarGetCell(lanes.get(), static_cast<int>(index))));
    }
    MatVariable crossings(Mat_VarRead(file.get(), "pedestrian_crossing"), Mat_VarFree);
    for (std::size_t index = 0; index < count(crossings.get()); ++index)
    {
        MapCrossing crossing;
        auto position_x = field(crossings.get(), "x", index);
        auto position_y = field(crossings.get(), "y", index);
        if (count(position_x) < 2 || count(position_x) != count(position_y))
        {
            throw std::runtime_error("pedestrian_crossing needs matching x/y centerline samples");
        }
        for (std::size_t point = 0; point < count(position_x); ++point)
        {
            crossing.centerline.push_back({number(position_x, point), number(position_y, point)});
        }
        auto width = field(crossings.get(), "width", index);
        if (count(width) > 0)
        {
            crossing.width = number(width, 0);
            if (crossing.width <= 0.0) {throw std::runtime_error("Crossing width must be positive");}
        }
        data.pedestrian_crossings.push_back(std::move(crossing));
    }
    loadActors(file.get(), "objects", 0, false, data, prediction_horizon, trim_zero_padding);
    loadActors(file.get(), "pedestrian", 7, false, data, prediction_horizon, trim_zero_padding);
    loadActors(file.get(), "cyclists", 6, false, data, prediction_horizon, trim_zero_padding);
    loadActors(file.get(), "obstacles", 0, true, data, prediction_horizon, trim_zero_padding);
    return data;
}

std::size_t ScenarioData::timeIndex(double elapsed) const
{
    if (!std::isfinite(elapsed)) {throw std::runtime_error("Elapsed time must be finite");}
    auto next = std::upper_bound(time.begin(), time.end(), std::max(0.0, elapsed) + 1e-9);
    return next == time.begin() ? 0 : static_cast<std::size_t>(next - time.begin() - 1);
}

double ScenarioData::duration() const
{
    return time.back();
}

}
}