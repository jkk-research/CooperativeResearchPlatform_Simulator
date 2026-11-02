#include "simulator/scenario_playback.hpp"

#include <boost/geometry.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <tf2/LinearMath/Quaternion.h>

namespace crp
{
namespace apl
{
namespace
{

namespace geometry = boost::geometry;
using Point = geometry::model::d2::point_xy<double>;
using Polygon = geometry::model::polygon<Point>;
using Object = autoware_perception_msgs::msg::PredictedObject;
using Line = geometry::model::linestring<Point>;

struct Route
{
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<double> distance;

    explicit Route(std::vector<geometry_msgs::msg::Point> samples) : points(std::move(samples))
    {
        for (std::size_t index = 0; index < points.size(); ++index)
        {
            distance.push_back(index == 0 ? 0.0 : distance.back() + std::hypot(
                points[index].x - points[index - 1].x, points[index].y - points[index - 1].y));
        }
    }

    double length() const {return distance.empty() ? 0.0 : distance.back();}

    geometry_msgs::msg::Pose pose(double progress, double direction) const
    {
        geometry_msgs::msg::Pose result;
        result.orientation.w = 1.0;
        if (points.size() < 2 || length() < 1e-8) {return result;}
        progress = std::clamp(progress, 0.0, length());
        const auto next = std::upper_bound(distance.begin(), distance.end(), progress);
        const auto index = std::min(static_cast<std::size_t>(next - distance.begin()), points.size() - 1);
        const auto & first = points[index - 1];
        const auto & second = points[index];
        const double segment = distance[index] - distance[index - 1];
        const double fraction = segment > 1e-8 ? (progress - distance[index - 1]) / segment : 0.0;
        result.position.x = first.x + fraction * (second.x - first.x);
        result.position.y = first.y + fraction * (second.y - first.y);
        result.position.z = first.z + fraction * (second.z - first.z);
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, -std::atan2(direction * (second.z - first.z), segment),
            std::atan2(direction * (second.y - first.y), direction * (second.x - first.x)));
        result.orientation.x = rotation.x(); result.orientation.y = rotation.y();
        result.orientation.z = rotation.z(); result.orientation.w = rotation.w();
        return result;
    }
};

Point position(const Object & object)
{
    const auto & point = object.kinematics.initial_pose_with_covariance.pose.position;
    return Point(point.x, point.y);
}

bool contains(const std::vector<Polygon> & polygons, const Point & point)
{
    return std::any_of(polygons.begin(), polygons.end(), [&](const auto & polygon)
    {
        return geometry::covered_by(point, polygon);
    });
}

Polygon footprint(const Object & object)
{
    Polygon result;
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    if (rotation.length2() < 1e-12) {rotation.setRPY(0.0, 0.0, 0.0);}
    else {rotation.normalize();}
    for (const auto & sign : std::array<std::array<double, 2>, 4>{{
        {{-1.0, -1.0}}, {{1.0, -1.0}}, {{1.0, 1.0}}, {{-1.0, 1.0}}}})
    {
        const auto offset = tf2::quatRotate(rotation, tf2::Vector3(
            sign[0] * 0.5 * object.shape.dimensions.x, sign[1] * 0.5 * object.shape.dimensions.y, 0.0));
        result.outer().emplace_back(pose.position.x + offset.x(), pose.position.y + offset.y());
    }
    geometry::correct(result);
    return result;
}

std::vector<Polygon> shadows(const Polygon & box, double range)
{
    std::vector<Polygon> result;
    if (geometry::covered_by(Point(0.0, 0.0), box)) {return result;}
    for (std::size_t index = 1; index < box.outer().size(); ++index)
    {
        const auto & first = box.outer()[index - 1];
        const auto & second = box.outer()[index];
        const double delta_x = second.x() - first.x();
        const double delta_y = second.y() - first.y();
        if (-delta_y * (first.x() + second.x()) + delta_x * (first.y() + second.y()) <= 1e-9)
        {
            continue;
        }
        const double first_distance = std::hypot(first.x(), first.y());
        const double second_distance = std::hypot(second.x(), second.y());
        const double extent = std::max({range, first_distance + 0.001, second_distance + 0.001});
        Polygon shadow;
        shadow.outer() = {first, second,
            Point(second.x() * extent / second_distance, second.y() * extent / second_distance),
            Point(first.x() * extent / first_distance, first.y() * extent / first_distance)};
        geometry::correct(shadow);
        if (geometry::area(shadow) > 1e-8) {result.push_back(std::move(shadow));}
    }
    return result;
}

bool largeObject(const Object & object)
{
    const auto & size = object.shape.dimensions;
    const bool vulnerable = std::any_of(object.classification.begin(), object.classification.end(),
        [](const auto & label) {return label.label == 6 || label.label == 7;});
    return !vulnerable && size.x >= 2.0 && size.y >= 1.0 && size.z >= 1.0;
}

std::vector<std::pair<double, double>> hiddenIntervals(const Route & route, const std::vector<Polygon> & regions)
{
    std::vector<std::pair<double, double>> intervals;
    for (std::size_t index = 1; index < route.points.size(); ++index)
    {
        const Point first(route.points[index - 1].x, route.points[index - 1].y);
        const Point second(route.points[index].x, route.points[index].y);
        const double length = route.distance[index] - route.distance[index - 1];
        if (length < 1e-8) {continue;}
        const Line segment{first, second};
        for (const auto & region : regions)
        {
            std::vector<Line> intersections;
            geometry::intersection(segment, region, intersections);
            for (const auto & intersection : intersections)
            {
                if (intersection.size() < 2) {continue;}
                const double start = geometry::distance(first, intersection.front());
                const double end = geometry::distance(first, intersection.back());
                intervals.emplace_back(route.distance[index - 1] + std::min(start, end),
                    route.distance[index - 1] + std::max(start, end));
            }
        }
    }
    std::sort(intervals.begin(), intervals.end());
    std::vector<std::pair<double, double>> merged;
    for (const auto & interval : intervals)
    {
        if (interval.second - interval.first < 1e-6) {continue;}
        if (!merged.empty() && interval.first <= merged.back().second + 1e-5)
        {
            merged.back().second = std::max(merged.back().second, interval.second);
        }
        else {merged.push_back(interval);}
    }
    return merged;
}

void appendVirtual(crp_msgs::msg::Scenario & scenario, const Route & route, double start, double direction,
    unsigned int label, double existence, const std::array<double, 2> & speeds, double first_confidence,
    std::size_t feature, std::size_t boundary, std::uint8_t source, const PlaybackSettings & settings,
    const std::vector<Polygon> & regions, const std::vector<Polygon> & footprints, double time_step)
{
    if (existence <= 0.0 || route.length() < 0.05 ||
        scenario.virtual_object_ids.size() >= settings.maximum_virtual_objects) {return;}
    const auto initial = route.pose(start, direction);
    const Point point(initial.position.x, initial.position.y);
    if (!contains(regions, point) || contains(footprints, point) ||
        std::hypot(point.x(), point.y()) > settings.scenario_distance) {return;}
    for (const auto & existing : scenario.local_moving_objects.objects)
    {
        if (existing.classification.empty() || existing.classification.front().label != label) {continue;}
        if (geometry::distance(position(existing), point) < (label == 7 ? 1.0 : 2.0)) {return;}
    }
    Object object;
    for (std::size_t byte = 0; byte < 4; ++byte)
    {
        object.object_id.uuid[byte] = static_cast<std::uint8_t>(feature >> (8 * byte));
        object.object_id.uuid[byte + 4] = static_cast<std::uint8_t>(boundary >> (8 * byte));
    }
    object.object_id.uuid[8] = static_cast<std::uint8_t>(label);
    object.object_id.uuid[9] = source;
    object.object_id.uuid[15] = 0xF0;
    object.existence_probability = static_cast<float>(existence);
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = static_cast<std::uint8_t>(label);
    classification.probability = 1.0F;
    object.classification.push_back(classification);
    object.shape.type = label == 7 ? autoware_perception_msgs::msg::Shape::CYLINDER :
        autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = label == 7 ? 0.6 : 4.5;
    object.shape.dimensions.y = label == 7 ? 0.6 : 1.8;
    object.shape.dimensions.z = label == 7 ? 1.8 : 1.5;
    object.kinematics.initial_pose_with_covariance.pose = initial;
    object.kinematics.initial_twist_with_covariance.twist.linear.x =
        speeds[0] * first_confidence + speeds[1] * (1.0 - first_confidence);
    time_step = std::max(time_step, settings.prediction_horizon / 99.0);
    const auto nanoseconds = static_cast<std::int64_t>(std::llround(time_step * 1e9));
    for (std::size_t branch = 0; branch < speeds.size(); ++branch)
    {
        autoware_perception_msgs::msg::PredictedPath prediction;
        prediction.confidence = static_cast<float>(branch == 0 ? first_confidence : 1.0 - first_confidence);
        prediction.time_step.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
        prediction.time_step.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
        const auto count = std::min<std::size_t>(100,
            static_cast<std::size_t>(std::floor(settings.prediction_horizon / time_step + 1e-8)) + 1);
        for (std::size_t sample = 0; sample < count; ++sample)
        {
            prediction.path.push_back(route.pose(start + direction * speeds[branch] * sample * time_step, direction));
        }
        object.kinematics.predicted_paths.push_back(std::move(prediction));
    }
    scenario.virtual_object_ids.push_back(object.object_id);
    scenario.local_moving_objects.objects.push_back(std::move(object));
}

void generateVirtualObjects(crp_msgs::msg::Scenario & scenario, const PlaybackSettings & settings,
    const std::vector<Polygon> & regions, const std::vector<Polygon> & footprints, double time_step)
{
    const std::array<double, 2> pedestrian_speeds{{settings.pedestrian_walking_speed, settings.pedestrian_running_speed}};
    for (const auto & crossing : scenario.map_features.pedestrian_crossings)
    {
        const Route route(crossing.centerline);
        const auto intervals = hiddenIntervals(route, regions);
        for (std::size_t index = 0; index < intervals.size(); ++index)
        {
            const auto & interval = intervals[index];
            const double inset = std::min(0.25, 0.5 * (interval.second - interval.first));
            if (interval.first > 1e-5)
            {
                appendVirtual(scenario, route, interval.first + inset, -1.0, 7,
                    settings.crossing_pedestrian_probability, pedestrian_speeds, settings.pedestrian_walking_confidence,
                    crossing.id, 2 * index, 2, settings, regions, footprints, time_step);
            }
            if (interval.second < route.length() - 1e-5)
            {
                appendVirtual(scenario, route, interval.second - inset, 1.0, 7,
                    settings.crossing_pedestrian_probability, pedestrian_speeds, settings.pedestrian_walking_confidence,
                    crossing.id, 2 * index + 1, 2, settings, regions, footprints, time_step);
            }
        }
    }
    for (std::size_t lane = scenario.paths.size() > 1 ? 1 : 0; lane < scenario.paths.size(); ++lane)
    {
        const auto & corridor = scenario.paths[lane].path;
        const auto lane_identifier = !corridor.points.empty() && !corridor.points.front().lane_ids.empty() ?
            static_cast<std::size_t>(corridor.points.front().lane_ids.front()) : lane;
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto & point : corridor.points) {points.push_back(point.point.pose.position);}
        const Route route(points);
        const auto intervals = hiddenIntervals(route, regions);
        for (std::size_t index = 0; index < intervals.size(); ++index)
        {
            const auto & interval = intervals[index];
            if (interval.second >= route.length() - 1e-5) {continue;}
            appendVirtual(scenario, route, interval.second - std::min(0.25, 0.5 * (interval.second - interval.first)),
                1.0, 1, settings.virtual_vehicle_probability,
                {{settings.virtual_vehicle_fast_speed, settings.virtual_vehicle_slow_speed}},
                settings.virtual_vehicle_fast_confidence, lane_identifier, index, 1, settings, regions, footprints, time_step);
        }
        for (std::size_t side = 0; side < 2; ++side)
        {
            const Route edge(side == 0 ? corridor.left_bound : corridor.right_bound);
            for (const auto & interval : hiddenIntervals(edge, regions))
            {
                if (interval.second >= edge.length() - 1e-5) {continue;}
                const auto initial = edge.pose(interval.second - std::min(0.25,
                    0.5 * (interval.second - interval.first)), 1.0);
                bool near_crossing = false;
                for (const auto & crossing : scenario.map_features.pedestrian_crossings)
                {
                    Line centerline;
                    for (const auto & point : crossing.centerline) {centerline.emplace_back(point.x, point.y);}
                    if (centerline.size() >= 2 && geometry::distance(
                        Point(initial.position.x, initial.position.y), centerline) < 0.5 * crossing.width + 3.0)
                    {
                        near_crossing = true;
                        break;
                    }
                }
                if (near_crossing) {continue;}
                const auto nearest = route.pose(std::min(route.length(), interval.second), 1.0).position;
                const double delta_x = nearest.x - initial.position.x;
                const double delta_y = nearest.y - initial.position.y;
                const double length = std::hypot(delta_x, delta_y);
                if (length < 0.1) {continue;}
                auto destination = initial.position;
                destination.x += delta_x * settings.lane_width / length;
                destination.y += delta_y * settings.lane_width / length;
                destination.z += (nearest.z - initial.position.z) * settings.lane_width / length;
                const Route step_in({initial.position, destination});
                appendVirtual(scenario, step_in, 0.0, 1.0, 7, settings.roadside_pedestrian_probability,
                    pedestrian_speeds, settings.pedestrian_walking_confidence, lane_identifier,
                    side, 3, settings, regions, footprints, time_step);
                break;
            }
        }
    }
}

}

void ScenarioPlayback::applyOcclusion(crp_msgs::msg::Scenario & scenario) const
{
    std::vector<const Object *> occluders;
    for (const auto * objects : {&scenario.local_moving_objects.objects, &scenario.local_obstacles.objects})
    {
        for (const auto & object : *objects)
        {
            if (largeObject(object)) {occluders.push_back(&object);}
        }
    }
    std::stable_sort(occluders.begin(), occluders.end(), [](const auto * first, const auto * second)
    {
        return geometry::comparable_distance(position(*first), Point(0.0, 0.0)) <
               geometry::comparable_distance(position(*second), Point(0.0, 0.0));
    });
    std::vector<Polygon> regions;
    std::vector<Polygon> footprints;
    std::vector<unique_identifier_msgs::msg::UUID> visible_occluders;
    for (const auto * object : occluders)
    {
        if (contains(regions, position(*object))) {continue;}
        visible_occluders.push_back(object->object_id);
        footprints.push_back(footprint(*object));
        for (const auto & region : shadows(footprint(*object), m_settings.object_distance))
        {
            geometry_msgs::msg::Polygon message;
            const auto & pose = object->kinematics.initial_pose_with_covariance.pose;
            for (std::size_t index = 0; index + 1 < region.outer().size(); ++index)
            {
                geometry_msgs::msg::Point32 point;
                point.x = static_cast<float>(region.outer()[index].x());
                point.y = static_cast<float>(region.outer()[index].y());
                point.z = static_cast<float>(pose.position.z);
                message.points.push_back(point);
            }
            scenario.occluded_regions.push_back(std::move(message));
            regions.push_back(region);
        }
    }
    for (auto * objects : {&scenario.local_moving_objects.objects, &scenario.local_obstacles.objects})
    {
        objects->erase(std::remove_if(objects->begin(), objects->end(), [&](const auto & object)
        {
            if (std::find(visible_occluders.begin(), visible_occluders.end(), object.object_id) !=
                visible_occluders.end()) {return false;}
            return contains(regions, position(object));
        }), objects->end());
    }
    if (m_settings.generate_virtual_objects && !regions.empty())
    {
        generateVirtualObjects(scenario, m_settings, regions, footprints, m_data.time_step);
    }
}

}
}