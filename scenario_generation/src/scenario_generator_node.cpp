#include "scenario_generation/scenario_playback.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/scenario_visualization.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace crp
{
namespace apl
{

class ScenarioGenerator : public rclcpp::Node
{
public:
    ScenarioGenerator() : Node("scenario_generator")
    {
        const auto positive = [this](const std::string & name, double fallback)
        {
            const double value = declare_parameter<double>(name, fallback);
            if (!std::isfinite(value) || value <= 0.0)
            {
                throw std::runtime_error(name + " must be finite and positive");
            }
            return value;
        };
        const auto finite = [this](const std::string & name, double fallback)
        {
            const double value = declare_parameter<double>(name, fallback);
            if (!std::isfinite(value)) {throw std::runtime_error(name + " must be finite");}
            return value;
        };
        PlaybackSettings settings;
        settings.scenario_distance = positive("scenario_distance", settings.scenario_distance);
        settings.scenario_lookahead_time = finite("scenario_lookahead_time", settings.scenario_lookahead_time);
        if (settings.scenario_lookahead_time < 0.0)
        {
            throw std::runtime_error("scenario_lookahead_time must be nonnegative");
        }
        settings.lane_width = positive("lane_width", settings.lane_width);
        settings.target_path_width = positive("target_path_width", settings.target_path_width);
        settings.crossing_width = positive("crossing_width", settings.crossing_width);
        settings.prediction_horizon = positive("prediction_horizon", settings.prediction_horizon);
        settings.object_distance = positive("object_distance", settings.object_distance);
        settings.enable_occlusion = declare_parameter<bool>("enable_occlusion", settings.enable_occlusion);
        settings.generate_virtual_objects = declare_parameter<bool>(
            "generate_virtual_objects", settings.generate_virtual_objects);
        settings.crossing_pedestrian_probability = finite(
            "crossing_pedestrian_probability", settings.crossing_pedestrian_probability);
        settings.roadside_pedestrian_probability = finite(
            "roadside_pedestrian_probability", settings.roadside_pedestrian_probability);
        settings.virtual_vehicle_probability = finite(
            "virtual_vehicle_probability", settings.virtual_vehicle_probability);
        settings.pedestrian_walking_speed = positive(
            "pedestrian_walking_speed", settings.pedestrian_walking_speed);
        settings.pedestrian_running_speed = positive(
            "pedestrian_running_speed", settings.pedestrian_running_speed);
        settings.pedestrian_walking_confidence = finite(
            "pedestrian_walking_confidence", settings.pedestrian_walking_confidence);
        settings.virtual_vehicle_fast_speed = positive(
            "virtual_vehicle_fast_speed", settings.virtual_vehicle_fast_speed);
        settings.virtual_vehicle_slow_speed = positive(
            "virtual_vehicle_slow_speed", settings.virtual_vehicle_slow_speed);
        settings.virtual_vehicle_fast_confidence = finite(
            "virtual_vehicle_fast_confidence", settings.virtual_vehicle_fast_confidence);
        const auto maximum_virtual_objects = declare_parameter<std::int64_t>(
            "maximum_virtual_objects", static_cast<std::int64_t>(settings.maximum_virtual_objects));
        if (maximum_virtual_objects <= 0) {throw std::runtime_error("maximum_virtual_objects must be positive");}
        settings.maximum_virtual_objects = static_cast<std::size_t>(maximum_virtual_objects);

        const auto default_scenario = ament_index_cpp::get_package_share_directory("scenario_generation") +
            "/scenario_files/occluded_pedestrian_costmap_data.mat";
        const auto scenario_file = declare_parameter<std::string>("scenario_file", default_scenario);
        const bool trim_zero_padding = declare_parameter<bool>("trim_zero_padding", true);
        m_playback = std::make_unique<ScenarioPlayback>(
            ScenarioData::load(scenario_file, settings.prediction_horizon, trim_zero_padding), settings);

        const auto displacement = finite("initial_displacement", 0.0);
        if (displacement < 0.0) {throw std::runtime_error("initial_displacement must be nonnegative");}
        const auto initial_pose = pathPoseAtDistance(m_playback->data().target_path, displacement);
        const auto unset = std::numeric_limits<double>::quiet_NaN();
        const auto initial_x = declare_parameter<double>("initial_x", unset);
        const auto initial_y = declare_parameter<double>("initial_y", unset);
        const auto initial_yaw = declare_parameter<double>("initial_yaw", unset);
        if (std::isinf(initial_x) || std::isinf(initial_y) || std::isinf(initial_yaw))
        {
            throw std::runtime_error("initial_x, initial_y and initial_yaw must be finite or unset");
        }
        m_local_initial_x = std::isfinite(initial_x) ? initial_x : initial_pose.point.x;
        m_local_initial_y = std::isfinite(initial_y) ? initial_y : initial_pose.point.y;
        m_local_initial_yaw = std::isfinite(initial_yaw) ? initial_yaw : initial_pose.heading;

        m_publish_ego_tf = declare_parameter<bool>("publish_ego_tf", true);
        m_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        m_dynamic_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        m_scenario_pub = create_publisher<crp_msgs::msg::Scenario>("/scenario", 10);
        m_visualization_pub = create_publisher<crp_msgs::msg::ScenarioVisualization>(
            "/scenario_visualization", 10);
        m_tracking_pub = create_publisher<crp_msgs::msg::Scenario>("/simulator/tracking_scenario", 1);
        m_ego_sub = create_subscription<crp_msgs::msg::Ego>("/ego", 10,
            [this](crp_msgs::msg::Ego::ConstSharedPtr ego) {onEgo(*ego);});

        RCLCPP_INFO(get_logger(), "Loaded %s: %zu lanes, %zu objects, %.2f seconds",
            scenario_file.c_str(), m_playback->data().lanes.size(),
            m_playback->data().actors.size(), m_playback->data().duration());
    }

private:
    static double yaw(const geometry_msgs::msg::Quaternion & orientation)
    {
        return std::atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    }

    void initializeMapAlignment(const crp_msgs::msg::Ego & ego, double global_yaw)
    {
        m_alignment_yaw = global_yaw - m_local_initial_yaw;
        const double cosine = std::cos(m_alignment_yaw);
        const double sine = std::sin(m_alignment_yaw);
        m_map_origin_x = ego.pose.pose.position.x -
            (cosine * m_local_initial_x - sine * m_local_initial_y);
        m_map_origin_y = ego.pose.pose.position.y -
            (sine * m_local_initial_x + cosine * m_local_initial_y);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = ego.header.stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "map_local";
        transform.transform.translation.x = m_map_origin_x;
        transform.transform.translation.y = m_map_origin_y;
        transform.transform.rotation.z = std::sin(0.5 * m_alignment_yaw);
        transform.transform.rotation.w = std::cos(0.5 * m_alignment_yaw);
        m_static_broadcaster->sendTransform(transform);
        m_alignment_initialized = true;
    }

    void publishMapEgoTransform(const crp_msgs::msg::Ego & ego)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = ego.header.stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = ego.pose.pose.position.x;
        transform.transform.translation.y = ego.pose.pose.position.y;
        transform.transform.translation.z = ego.pose.pose.position.z;
        transform.transform.rotation = ego.pose.pose.orientation;
        m_dynamic_broadcaster->sendTransform(transform);
    }

    ScenarioEgoState localEgo(const crp_msgs::msg::Ego & ego, const std::string & source_frame)
    {
        ScenarioEgoState result;
        const double ego_yaw = yaw(ego.pose.pose.orientation);
        if (source_frame == "map_local")
        {
            result.x = ego.pose.pose.position.x;
            result.y = ego.pose.pose.position.y;
            result.yaw = ego_yaw;
        }
        else if (source_frame == "map")
        {
            if (!m_alignment_initialized) {initializeMapAlignment(ego, ego_yaw);}
            const double delta_x = ego.pose.pose.position.x - m_map_origin_x;
            const double delta_y = ego.pose.pose.position.y - m_map_origin_y;
            result.x = std::cos(m_alignment_yaw) * delta_x + std::sin(m_alignment_yaw) * delta_y;
            result.y = -std::sin(m_alignment_yaw) * delta_x + std::cos(m_alignment_yaw) * delta_y;
            result.yaw = ego_yaw - m_alignment_yaw;
        }
        else
        {
            throw std::runtime_error("Ego frame must be map or map_local");
        }
        const auto road = m_playback->roadState(result);
        result.z = road.elevation;
        result.pitch = road.pitch;
        return result;
    }

    void onEgo(const crp_msgs::msg::Ego & ego)
    {
        const auto source_frame = ego.header.frame_id.empty() ? std::string("map") : ego.header.frame_id;
        if (source_frame != m_source_frame)
        {
            if (!m_source_frame.empty())
            {
                RCLCPP_ERROR(get_logger(), "Ego frame changed from '%s' to '%s' during scenario playback",
                    m_source_frame.c_str(), source_frame.c_str());
                return;
            }
            m_source_frame = source_frame;
        }
        ScenarioEgoState local;
        try {local = localEgo(ego, source_frame);}
        catch (const std::exception & error)
        {
            RCLCPP_ERROR(get_logger(), "%s", error.what());
            return;
        }
        if (m_publish_ego_tf && source_frame == "map") {publishMapEgoTransform(ego);}
        rclcpp::Time stamp(ego.header.stamp);
        if (!m_started)
        {
            m_start_stamp = stamp;
            m_started = true;
        }
        m_elapsed = std::max(m_elapsed, std::max(0.0, (stamp - m_start_stamp).seconds()));
        auto scenario = m_playback->message(local, m_elapsed);
        scenario.header.stamp = ego.header.stamp;
        scenario.local_moving_objects.header.stamp = ego.header.stamp;
        scenario.local_obstacles.header.stamp = ego.header.stamp;
        for (auto & path : scenario.paths)
        {
            path.header.stamp = ego.header.stamp;
            path.path.header.stamp = ego.header.stamp;
            path.traffic_rules.header.stamp = ego.header.stamp;
        }
        publishTrackingScenario(scenario);
        m_scenario_pub->publish(scenario);
        crp_msgs::msg::ScenarioVisualization visualization;
        visualization.header = scenario.header;
        visualization.scenario = std::move(scenario);
        m_visualization_pub->publish(visualization);
    }

    void publishTrackingScenario(const crp_msgs::msg::Scenario & scenario)
    {
        crp_msgs::msg::Scenario tracking;
        tracking.header = scenario.header;
        tracking.local_moving_objects = scenario.local_moving_objects;
        tracking.local_obstacles = scenario.local_obstacles;
        tracking.virtual_object_ids = scenario.virtual_object_ids;
        if (!scenario.paths.empty() && scenario.paths.front().path.points.size() >= 2)
        {
            const auto & source = scenario.paths.front();
            crp_msgs::msg::PathWithTrafficRules target;
            target.header = source.header;
            target.path.header = source.path.header;
            target.traffic_rules = source.traffic_rules;
            target.path.points = {source.path.points.front(), source.path.points.back()};
            if (!source.path.left_bound.empty() && !source.path.right_bound.empty())
            {
                target.path.left_bound = {source.path.left_bound.front(), source.path.left_bound.back()};
                target.path.right_bound = {source.path.right_bound.front(), source.path.right_bound.back()};
            }
            tracking.paths.push_back(std::move(target));
        }
        m_tracking_pub->publish(tracking);
    }

    std::unique_ptr<ScenarioPlayback> m_playback;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_static_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_dynamic_broadcaster;
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_scenario_pub;
    rclcpp::Publisher<crp_msgs::msg::ScenarioVisualization>::SharedPtr m_visualization_pub;
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_tracking_pub;
    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr m_ego_sub;
    rclcpp::Time m_start_stamp{0, 0, RCL_ROS_TIME};
    std::string m_source_frame;
    double m_local_initial_x{0.0};
    double m_local_initial_y{0.0};
    double m_local_initial_yaw{0.0};
    double m_map_origin_x{0.0};
    double m_map_origin_y{0.0};
    double m_alignment_yaw{0.0};
    double m_elapsed{0.0};
    bool m_started{false};
    bool m_alignment_initialized{false};
    bool m_publish_ego_tf{true};
};

}
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<crp::apl::ScenarioGenerator>());
    }
    catch (const std::exception & error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("scenario_generator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}