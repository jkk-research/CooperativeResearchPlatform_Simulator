#include "simulator/scenario_playback.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace crp
{
namespace apl
{
namespace
{

geometry_msgs::msg::Pose localPose(const MapPoint & point, double heading, const VehicleState & ego)
{
    geometry_msgs::msg::Pose pose;
    const double delta_x = point.x - ego.x;
    const double delta_y = point.y - ego.y;
    pose.position.x = delta_x * std::cos(ego.yaw) + delta_y * std::sin(ego.yaw);
    pose.position.y = -delta_x * std::sin(ego.yaw) + delta_y * std::cos(ego.yaw);
    pose.orientation.z = std::sin(0.5 * (heading - ego.yaw));
    pose.orientation.w = std::cos(0.5 * (heading - ego.yaw));
    return pose;
}

std::size_t nearestPoint(const std::vector<MapPoint> & points, const VehicleState & ego)
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
    const VehicleState & ego, double fallback_heading)
{
    autoware_perception_msgs::msg::PredictedPath result;
    const auto nanoseconds = static_cast<std::int64_t>(std::llround(time_step * 1e9));
    result.time_step.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
    result.time_step.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
    result.confidence = static_cast<float>(confidence);
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        result.path.push_back(localPose(points[index],
            points.size() > 1 ? pathHeading(points, index) : fallback_heading, ego));
    }
    return result;
}

}

ScenarioPlayback::ScenarioPlayback(ScenarioData data, PlaybackSettings settings)
: m_data(std::move(data)), m_settings(settings)
{
}

const ScenarioData & ScenarioPlayback::data() const {return m_data;}

std::size_t ScenarioPlayback::nearestTarget(const VehicleState & ego) const
{
    return nearestPoint(m_data.target_path, ego);
}

double ScenarioPlayback::speedLimit(const VehicleState & ego) const
{
    return m_data.speed_limits[nearestTarget(ego)];
}

crp_msgs::msg::Scenario ScenarioPlayback::message(const VehicleState & ego, double elapsed) const
{
    crp_msgs::msg::Scenario result;
    result.header.frame_id = "base_link";
    result.local_moving_objects.header.frame_id = "base_link";
    result.local_obstacles.header.frame_id = "base_link";
    const auto target_index = nearestTarget(ego);
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
        while (begin > 0 && backward < m_settings.scenario_distance)
        {
            backward += std::hypot(points[begin].x - points[begin - 1].x,
                points[begin].y - points[begin - 1].y);
            --begin;
        }
        double forward = 0.0;
        while (end < points.size() && forward < m_settings.scenario_distance)
        {
            forward += std::hypot(points[end].x - points[end - 1].x,
                points[end].y - points[end - 1].y);
            ++end;
        }
        for (std::size_t index = begin; index < end; ++index)
        {
            if (std::hypot(points[index].x - ego.x, points[index].y - ego.y) > m_settings.scenario_distance)
            {
                continue;
            }
            const auto heading = pathHeading(points, index);
            tier4_planning_msgs::msg::PathPointWithLaneId point;
            point.point.pose = localPose(points[index], heading, ego);
            point.point.longitudinal_velocity_mps = static_cast<float>(
                target ? m_data.speed_limits[index] : m_data.speed_limits[target_index]);
            point.lane_ids.push_back(static_cast<std::int64_t>(lane_id));
            corridor.path.points.push_back(point);
            const MapPoint left{points[index].x - 0.5 * width * std::sin(heading),
                points[index].y + 0.5 * width * std::cos(heading)};
            const MapPoint right{points[index].x + 0.5 * width * std::sin(heading),
                points[index].y - 0.5 * width * std::cos(heading)};
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
                return std::hypot(point.x - ego.x, point.y - ego.y) <= m_settings.scenario_distance;
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
        object.existence_probability = static_cast<float>(actor.existence_probabilities[observation]);
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
            localPose(actor.positions[observation], actor.headings[observation], ego);
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
            m_data.time_step, ego, actor.headings[observation]));
        for (const auto & alternative : actor.alternatives[observation])
        {
            predictions.push_back(predictedPath(alternative.points, alternative.confidence,
                m_data.time_step, ego, actor.headings[observation]));
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
    return result;
}

}
}