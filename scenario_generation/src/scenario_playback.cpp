#include "scenario_generation/scenario_playback.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <tf2/LinearMath/Transform.h>

namespace crp
{
namespace apl
{
namespace
{

geometry_msgs::msg::Pose localPose(const MapPoint & point, double heading, const ScenarioEgoState & ego,
    double pitch = 0.0)
{
    geometry_msgs::msg::Pose pose;
    tf2::Quaternion ego_rotation;
    ego_rotation.setRPY(0.0, -ego.pitch, ego.yaw);
    const tf2::Transform ego_transform(ego_rotation, tf2::Vector3(ego.x, ego.y, ego.z));
    const auto local = ego_transform.inverse() * tf2::Vector3(point.x, point.y, point.z);
    pose.position.x = local.x(); pose.position.y = local.y(); pose.position.z = local.z();
    tf2::Quaternion map_rotation;
    map_rotation.setRPY(0.0, -pitch, heading);
    const auto rotation = ego_rotation.inverse() * map_rotation;
    pose.orientation.x = rotation.x(); pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z(); pose.orientation.w = rotation.w();
    return pose;
}

std::size_t nearestPoint(const std::vector<MapPoint> & points, const ScenarioEgoState & ego)
{
    std::size_t nearest = 0;
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const double distance = std::hypot(points[index].x - ego.x, points[index].y - ego.y);
        if (distance < minimum) {minimum = distance; nearest = index;}
    }
    return nearest;
}

autoware_perception_msgs::msg::PredictedPath predictedPath(
    const std::vector<MapPoint> & points, double confidence, double time_step,
    const ScenarioEgoState & ego, double fallback_heading, double fallback_pitch)
{
    autoware_perception_msgs::msg::PredictedPath result;
    const auto nanoseconds = static_cast<std::int64_t>(std::llround(time_step * 1e9));
    result.time_step.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
    result.time_step.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
    result.confidence = static_cast<float>(confidence);
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        result.path.push_back(localPose(points[index],
            points.size() > 1 ? pathHeading(points, index) : fallback_heading, ego,
            points.size() > 1 ? pathPitch(points, index) : fallback_pitch));
    }
    return result;
}

}

ScenarioPlayback::ScenarioPlayback(ScenarioData data, PlaybackSettings settings)
: m_data(std::move(data)), m_settings(settings)
{
    if (!std::isfinite(settings.scenario_distance) || settings.scenario_distance <= 0.0 ||
        !std::isfinite(settings.scenario_lookahead_time) || settings.scenario_lookahead_time < 0.0)
    {
        throw std::runtime_error("Invalid scenario distance or lookahead time");
    }
    for (const auto value : {settings.crossing_pedestrian_probability, settings.roadside_pedestrian_probability,
        settings.virtual_vehicle_probability, settings.pedestrian_walking_confidence,
        settings.virtual_vehicle_fast_confidence})
    {
        if (!std::isfinite(value) || value < 0.0 || value > 1.0)
        {
            throw std::runtime_error("Virtual-object probabilities must be finite and between zero and one");
        }
    }
    for (const auto value : {settings.pedestrian_walking_speed, settings.pedestrian_running_speed,
        settings.virtual_vehicle_fast_speed, settings.virtual_vehicle_slow_speed})
    {
        if (!std::isfinite(value) || value <= 0.0)
        {
            throw std::runtime_error("Virtual-object speeds must be finite and positive");
        }
    }
    if (settings.maximum_virtual_objects == 0) {throw std::runtime_error("maximum_virtual_objects must be positive");}
}

const ScenarioData & ScenarioPlayback::data() const {return m_data;}

std::size_t ScenarioPlayback::nearestTarget(const ScenarioEgoState & ego) const
{
    return nearestPoint(m_data.target_path, ego);
}

double ScenarioPlayback::speedLimit(const ScenarioEgoState & ego) const
{
    return m_data.speed_limits[nearestTarget(ego)];
}

double ScenarioPlayback::scenarioDistance(const ScenarioEgoState & ego) const
{
    return std::max(m_settings.scenario_distance,
        speedLimit(ego) * m_settings.scenario_lookahead_time);
}

RoadState ScenarioPlayback::roadState(const ScenarioEgoState & ego) const
{
    RoadState result;
    double minimum = std::numeric_limits<double>::infinity();
    const auto & route = m_data.target_path;
    for (std::size_t index = 1; index < route.size(); ++index)
    {
        const auto & first = route[index - 1];
        const auto & second = route[index];
        const double delta_x = second.x - first.x;
        const double delta_y = second.y - first.y;
        const double squared_length = delta_x * delta_x + delta_y * delta_y;
        if (squared_length < 1e-12) {continue;}
        const double fraction = std::clamp(((ego.x - first.x) * delta_x +
            (ego.y - first.y) * delta_y) / squared_length, 0.0, 1.0);
        const double distance = std::hypot(first.x + fraction * delta_x - ego.x,
            first.y + fraction * delta_y - ego.y);
        if (distance >= minimum) {continue;}
        minimum = distance;
        result.elevation = first.z + fraction * (second.z - first.z);
        const double heading = std::atan2(delta_y, delta_x);
        result.pitch = std::atan((second.z - first.z) / std::sqrt(squared_length) *
            std::cos(ego.yaw - heading));
    }
    return result;
}

crp_msgs::msg::Scenario ScenarioPlayback::message(const ScenarioEgoState & ego, double elapsed) const
{
    crp_msgs::msg::Scenario result;
    result.header.frame_id = "base_link";
    result.local_moving_objects.header.frame_id = "base_link";
    result.local_obstacles.header.frame_id = "base_link";
    const auto target_index = nearestTarget(ego);
    const double scenario_distance = scenarioDistance(ego);
    const auto append_corridor = [&](const std::vector<MapPoint> & points, double width,
        bool target, std::size_t lane_id)
    {
        crp_msgs::msg::PathWithTrafficRules corridor;
        corridor.header.frame_id = "base_link";
        corridor.path.header.frame_id = "base_link";
        corridor.traffic_rules.header.frame_id = "base_link";
        corridor.traffic_rules.maximum_speed = static_cast<float>(m_data.speed_limits[target_index]);
        const auto nearest = nearestPoint(points, ego);
        std::size_t begin = nearest;
        std::size_t end = nearest + 1;
        double backward = 0.0;
        while (begin > 0 && backward < scenario_distance)
        {
            backward += std::hypot(points[begin].x - points[begin - 1].x,
                points[begin].y - points[begin - 1].y);
            --begin;
        }
        double forward = 0.0;
        while (end < points.size() && forward < scenario_distance)
        {
            forward += std::hypot(points[end].x - points[end - 1].x,
                points[end].y - points[end - 1].y);
            ++end;
        }
        for (std::size_t index = begin; index < end; ++index)
        {
            if (std::hypot(points[index].x - ego.x, points[index].y - ego.y) > scenario_distance)
            {
                continue;
            }
            const auto heading = pathHeading(points, index);
            tier4_planning_msgs::msg::PathPointWithLaneId point;
            point.point.pose = localPose(points[index], heading, ego, pathPitch(points, index));
            point.point.longitudinal_velocity_mps = static_cast<float>(
                target ? m_data.speed_limits[index] : m_data.speed_limits[target_index]);
            point.lane_ids.push_back(static_cast<std::int64_t>(lane_id));
            corridor.path.points.push_back(point);
            const MapPoint left{points[index].x - 0.5 * width * std::sin(heading),
                points[index].y + 0.5 * width * std::cos(heading), points[index].z};
            const MapPoint right{points[index].x + 0.5 * width * std::sin(heading),
                points[index].y - 0.5 * width * std::cos(heading), points[index].z};
            corridor.path.left_bound.push_back(localPose(left, heading, ego).position);
            corridor.path.right_bound.push_back(localPose(right, heading, ego).position);
        }
        if (target || !corridor.path.points.empty()) {result.paths.push_back(std::move(corridor));}
    };
    append_corridor(m_data.target_path, m_settings.target_path_width, true, 0);
    for (std::size_t lane = 0; lane < m_data.lanes.size(); ++lane)
    {
        append_corridor(m_data.lanes[lane], m_settings.lane_width, false, lane + 1);
    }

    for (std::size_t index = 0; index < m_data.pedestrian_crossings.size(); ++index)
    {
        const auto & crossing = m_data.pedestrian_crossings[index];
        const bool visible = std::any_of(crossing.centerline.begin(), crossing.centerline.end(),
            [&](const auto & point)
            {
                return std::hypot(point.x - ego.x, point.y - ego.y) <= scenario_distance;
            });
        if (!visible) {continue;}
        crp_msgs::msg::PedestrianCrossing message;
        message.id = static_cast<std::uint32_t>(index + 1);
        message.width = static_cast<float>(crossing.width > 0.0 ? crossing.width : m_settings.crossing_width);
        for (const auto & point : crossing.centerline)
        {
            message.centerline.push_back(localPose(point, 0.0, ego).position);
        }
        result.map_features.pedestrian_crossings.push_back(std::move(message));
    }

    const auto observation = m_data.timeIndex(elapsed);
    for (std::size_t actor_index = 0; actor_index < m_data.actors.size(); ++actor_index)
    {
        const auto & actor = m_data.actors[actor_index];
        if (observation >= actor.positions.size() || actor.existence_probabilities[observation] == 0.0)
        {
            continue;
        }
        if (std::hypot(actor.positions[observation].x - ego.x,
            actor.positions[observation].y - ego.y) > m_settings.object_distance) {continue;}
        autoware_perception_msgs::msg::PredictedObject object;
        const std::uint64_t identifier = actor_index + 1;
        for (std::size_t byte = 0; byte < sizeof(identifier); ++byte)
        {
            object.object_id.uuid[byte] = static_cast<std::uint8_t>(identifier >> (8 * byte));
        }
        object.existence_probability = 1.0F;
        autoware_perception_msgs::msg::ObjectClassification classification;
        classification.label = static_cast<std::uint8_t>(actor.classification);
        classification.probability = 1.0F;
        object.classification.push_back(classification);
        object.shape.type = actor.classification == 7 ?
            autoware_perception_msgs::msg::Shape::CYLINDER : autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
        object.shape.dimensions.x = actor.dimensions[0];
        object.shape.dimensions.y = actor.dimensions[1];
        object.shape.dimensions.z = actor.dimensions[2];
        object.kinematics.initial_pose_with_covariance.pose =
            localPose(actor.positions[observation], actor.headings[observation], ego,
                pathPitch(actor.positions, observation));
        object.kinematics.initial_twist_with_covariance.twist.linear.x = actor.speeds[observation];
        if (observation > 0)
        {
            object.kinematics.initial_acceleration_with_covariance.accel.linear.x =
                (actor.speeds[observation] - actor.speeds[observation - 1]) / m_data.time_step;
        }
        const auto end = std::min(actor.positions.size(), observation +
            static_cast<std::size_t>(std::floor(m_settings.prediction_horizon / m_data.time_step + 1e-8)) + 1);
        std::vector<MapPoint> main_points(actor.positions.begin() + observation, actor.positions.begin() + end);
        auto & predictions = object.kinematics.predicted_paths;
        predictions.push_back(predictedPath(main_points, actor.main_confidences[observation],
            m_data.time_step, ego, actor.headings[observation], pathPitch(actor.positions, observation)));
        for (const auto & alternative : actor.alternatives[observation])
        {
            predictions.push_back(predictedPath(alternative.points, alternative.confidence,
                m_data.time_step, ego, actor.headings[observation], pathPitch(actor.positions, observation)));
        }
        double confidence_sum = 0.0;
        for (const auto & prediction : predictions) {confidence_sum += prediction.confidence;}
        if (confidence_sum > 0.0)
        {
            for (auto & prediction : predictions)
            {
                prediction.confidence = static_cast<float>(prediction.confidence / confidence_sum);
            }
        }
        else
        {
            predictions.front().confidence = 1.0F;
        }
        if (actor.obstacle) {result.local_obstacles.objects.push_back(std::move(object));}
        else {result.local_moving_objects.objects.push_back(std::move(object));}
    }
    if (m_settings.enable_occlusion) {applyOcclusion(result);}
    return result;
}

}
}