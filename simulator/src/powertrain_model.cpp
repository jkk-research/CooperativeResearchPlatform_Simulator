#include "simulator/powertrain_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace crp
{
namespace apl
{
namespace
{

std::vector<std::string> split(const std::string & line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {fields.push_back(field);}
    if (!fields.empty() && !fields.back().empty() && fields.back().back() == '\r')
    {
        fields.back().pop_back();
    }
    return fields;
}

double interpolate(const std::vector<double> & grid, double value, std::size_t & lower)
{
    value = std::clamp(value, grid.front(), grid.back());
    const auto upper = std::upper_bound(grid.begin(), grid.end(), value);
    lower = upper == grid.begin() ? 0 :
        std::min(static_cast<std::size_t>(upper - grid.begin() - 1), grid.size() - 2);
    return (value - grid[lower]) / (grid[lower + 1] - grid[lower]);
}

}

PowertrainModel::PowertrainModel(const std::string & efficiency_map_path)
{
    std::ifstream input(efficiency_map_path);
    if (!input) {throw std::runtime_error("Unable to open motor efficiency map: " + efficiency_map_path);}
    std::string line;
    if (!std::getline(input, line)) {throw std::runtime_error("Motor efficiency map is empty");}
    const auto header = split(line);
    if (header.size() < 5 || header[0] != "mode" || header[1] != "speed_rpm")
    {
        throw std::runtime_error("Invalid motor efficiency map header");
    }
    for (std::size_t index = 2; index < header.size(); ++index)
    {
        constexpr char prefix[] = "torque_";
        constexpr char suffix[] = "_nm";
        const auto prefix_size = sizeof(prefix) - 1;
        const auto suffix_size = sizeof(suffix) - 1;
        if (header[index].size() <= prefix_size + suffix_size ||
            header[index].compare(0, prefix_size, prefix) != 0 ||
            header[index].compare(header[index].size() - suffix_size, suffix_size, suffix) != 0)
        {
            throw std::runtime_error("Invalid torque axis in motor efficiency map");
        }
        const double torque = std::stod(header[index].substr(prefix_size,
            header[index].size() - prefix_size - suffix_size));
        if (!std::isfinite(torque) || torque < 0.0 ||
            (!m_torque_grid.empty() && torque <= m_torque_grid.back()))
        {
            throw std::runtime_error("Motor efficiency torque axis must increase");
        }
        m_torque_grid.push_back(torque);
    }

    std::vector<double> regeneration_speeds;
    while (std::getline(input, line))
    {
        if (line.empty()) {continue;}
        const auto fields = split(line);
        if (fields.size() != header.size()) {throw std::runtime_error("Invalid motor efficiency map row");}
        const double speed = std::stod(fields[1]);
        if (!std::isfinite(speed) || speed < 0.0) {throw std::runtime_error("Invalid motor speed axis");}
        const bool motoring = fields[0] == "motoring";
        if (!motoring && fields[0] != "regeneration")
        {
            throw std::runtime_error("Motor efficiency map mode must be motoring or regeneration");
        }
        auto & speeds = motoring ? m_speed_grid : regeneration_speeds;
        auto & table = motoring ? m_motoring_efficiency : m_regeneration_efficiency;
        if (!speeds.empty() && speed <= speeds.back())
        {
            throw std::runtime_error("Motor speed axis must increase within each mode");
        }
        speeds.push_back(speed);
        std::vector<double> row;
        for (std::size_t index = 2; index < fields.size(); ++index)
        {
            const double value = std::stod(fields[index]);
            if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
            {
                throw std::runtime_error("Motor efficiencies must be in (0, 1]");
            }
            row.push_back(value);
        }
        table.push_back(std::move(row));
    }
    if (m_speed_grid.size() < 2 || regeneration_speeds != m_speed_grid ||
        m_motoring_efficiency.size() != m_speed_grid.size() ||
        m_regeneration_efficiency.size() != m_speed_grid.size())
    {
        throw std::runtime_error("Efficiency map needs matching motoring and regeneration speed grids");
    }
    for (std::size_t row = 0; row < m_speed_grid.size(); ++row)
    {
        if (m_motoring_efficiency[row].size() != m_torque_grid.size() ||
            m_regeneration_efficiency[row].size() != m_torque_grid.size())
        {
            throw std::runtime_error("Efficiency map tables must match the torque grid");
        }
    }
}

void PowertrainModel::validateSettings() const
{
    for (double value : {settings.motor_gear_ratio, settings.wheel_gear_ratio,
        settings.wheel_radius, settings.motor_inertia, settings.wheel_inertia,
        settings.shaft_stiffness, settings.maximum_motor_torque,
        settings.maximum_regenerative_torque, settings.motor_torque_rate_limit,
        settings.integration_step})
    {
        if (!std::isfinite(value) || value <= 0.0)
        {
            throw std::invalid_argument("Powertrain parameters must be finite and positive");
        }
    }
    for (double value : {settings.motor_viscous_friction, settings.wheel_viscous_friction})
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            throw std::invalid_argument("Powertrain friction must be finite and nonnegative");
        }
    }
    if (!std::isfinite(settings.shaft_damping) || settings.shaft_damping < 0.0)
    {
        throw std::invalid_argument("Shaft damping must be finite and nonnegative");
    }
}

void PowertrainModel::initialize(double vehicle_speed)
{
    validateSettings();
    if (!std::isfinite(vehicle_speed) || vehicle_speed < 0.0)
    {
        throw std::invalid_argument("Initial powertrain speed must be finite and nonnegative");
    }
    state = {};
    state.vehicle_speed = vehicle_speed;
    state.motor_speed = settings.motor_gear_ratio * settings.wheel_gear_ratio * vehicle_speed /
        settings.wheel_radius;
    state.efficiency = efficiency(state.motor_speed, 0.0, false);
}

double PowertrainModel::efficiency(double motor_speed, double motor_torque, bool regeneration) const
{
    const double rpm = std::abs(motor_speed) * 60.0 / (2.0 * std::acos(-1.0));
    const double torque = std::abs(motor_torque);
    std::size_t speed_index = 0;
    std::size_t torque_index = 0;
    const double speed_fraction = interpolate(m_speed_grid, rpm, speed_index);
    const double torque_fraction = interpolate(m_torque_grid, torque, torque_index);
    const auto & table = regeneration ? m_regeneration_efficiency : m_motoring_efficiency;
    const double low = table[speed_index][torque_index] * (1.0 - torque_fraction) +
        table[speed_index][torque_index + 1] * torque_fraction;
    const double high = table[speed_index + 1][torque_index] * (1.0 - torque_fraction) +
        table[speed_index + 1][torque_index + 1] * torque_fraction;
    return low * (1.0 - speed_fraction) + high * speed_fraction;
}

double PowertrainModel::step(double acceleration_target, double vehicle_speed, double time_step,
    const VehicleSettings & vehicle)
{
    validateSettings();
    if (!std::isfinite(acceleration_target) || !std::isfinite(vehicle_speed) || vehicle_speed < 0.0 ||
        !std::isfinite(time_step) || time_step <= 0.0)
    {
        throw std::invalid_argument("Invalid powertrain input");
    }
    const auto count = static_cast<std::size_t>(std::ceil(time_step / settings.integration_step));
    if (count > 100000) {throw std::invalid_argument("Powertrain update requires too many substeps");}
    const double step = time_step / static_cast<double>(count);
    const double effective_inertia = settings.motor_inertia * settings.motor_gear_ratio +
        settings.wheel_inertia / settings.motor_gear_ratio;
    const double effective_friction = settings.motor_gear_ratio * settings.motor_viscous_friction +
        settings.wheel_viscous_friction / settings.motor_gear_ratio;
    const double speed_ratio = settings.motor_gear_ratio * settings.wheel_gear_ratio /
        settings.wheel_radius;
    double force_integral = 0.0;

    for (std::size_t iteration = 0; iteration < count; ++iteration)
    {
        const double bounded_acceleration = std::clamp(acceleration_target,
            -vehicle.maximum_deceleration, vehicle.maximum_acceleration);
        const double requested_motor_acceleration = speed_ratio * bounded_acceleration;
        const double requested_shaft_twist = vehicle.mass * bounded_acceleration *
            settings.wheel_radius / (settings.shaft_stiffness * settings.wheel_gear_ratio);
        const double requested_torque = (effective_inertia * requested_motor_acceleration +
            effective_friction * state.motor_speed + settings.shaft_stiffness * requested_shaft_twist) /
            settings.motor_gear_ratio;
        const double bounded_torque = std::clamp(requested_torque, -settings.maximum_regenerative_torque,
            settings.maximum_motor_torque);
        const double maximum_torque_change = settings.motor_torque_rate_limit * step;
        state.motor_torque += std::clamp(bounded_torque - state.motor_torque,
            -maximum_torque_change, maximum_torque_change);
        const double mechanical_power = state.motor_torque * state.motor_speed;
        const bool regeneration = mechanical_power < 0.0;
        state.efficiency = efficiency(state.motor_speed, state.motor_torque, regeneration);
        state.electrical_power = regeneration ? mechanical_power * state.efficiency :
            mechanical_power / state.efficiency;
        state.energy_consumed += std::max(0.0, state.electrical_power) * step;
        state.energy_regenerated += std::max(0.0, -state.electrical_power) * step;

        const double twist_rate = state.motor_speed / settings.motor_gear_ratio -
            settings.wheel_gear_ratio * vehicle_speed / settings.wheel_radius;
        const double shaft_torque = settings.shaft_stiffness * state.shaft_twist +
            settings.shaft_damping * twist_rate;
        double wheel_force = settings.wheel_gear_ratio * shaft_torque / settings.wheel_radius;
        if (bounded_acceleration < 0.0)
        {
            wheel_force += std::min(0.0, vehicle.mass * bounded_acceleration - wheel_force);
        }
        force_integral += wheel_force * step;
        const double motor_derivative = (-effective_friction * state.motor_speed - shaft_torque +
            settings.motor_gear_ratio * state.motor_torque) /
            effective_inertia;
        const double twist_derivative = state.motor_speed / settings.motor_gear_ratio -
            settings.wheel_gear_ratio * vehicle_speed / settings.wheel_radius;
        state.motor_speed = std::max(0.0, state.motor_speed + step * motor_derivative);
        state.shaft_twist += step * twist_derivative;
    }
    state.vehicle_speed = vehicle_speed;
    return force_integral / time_step;
}

}
}