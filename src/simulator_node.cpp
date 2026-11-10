#include "simulator/scenario_playback.hpp"
#include "simulator/powertrain_model.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/ego_visualization.hpp>
#include <crp_msgs/msg/powertrain_telemetry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <crp_msgs/msg/scenario_visualization.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace crp
{
namespace apl
{

class Simulator : public rclcpp::Node
{
public:
    Simulator() : Node("simulator")
    {
        const auto configured_efficiency_map = declare_parameter<std::string>("motor_efficiency_map", "");
        const auto efficiency_map_path = configured_efficiency_map.empty() ?
            ament_index_cpp::get_package_share_directory("simulator") + "/config/motor_efficiency.csv" :
            configured_efficiency_map;
        m_powertrain = std::make_unique<PowertrainModel>(efficiency_map_path);
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
        const auto scenario_file = declare_parameter<std::string>("scenario_file", "");
        PlaybackSettings playback;
        playback.scenario_distance = positive("scenario_distance", playback.scenario_distance);
        playback.scenario_lookahead_time = finite("scenario_lookahead_time", playback.scenario_lookahead_time);
        if (playback.scenario_lookahead_time < 0.0)
        {
            throw std::runtime_error("scenario_lookahead_time must be nonnegative");
        }
        playback.lane_width = positive("lane_width", playback.lane_width);
        playback.target_path_width = positive("target_path_width", playback.target_path_width);
        playback.crossing_width = positive("crossing_width", playback.crossing_width);
        playback.prediction_horizon = positive("prediction_horizon", playback.prediction_horizon);
        playback.object_distance = positive("object_distance", playback.object_distance);
        playback.enable_occlusion = declare_parameter<bool>("enable_occlusion", true);
        playback.generate_virtual_objects = declare_parameter<bool>("generate_virtual_objects", true);
        playback.crossing_pedestrian_probability = finite("crossing_pedestrian_probability", 0.5);
        playback.roadside_pedestrian_probability = finite("roadside_pedestrian_probability", 0.02);
        playback.virtual_vehicle_probability = finite("virtual_vehicle_probability", 0.5);
        playback.pedestrian_walking_speed = positive("pedestrian_walking_speed", 0.8);
        playback.pedestrian_running_speed = positive("pedestrian_running_speed", 3.0);
        playback.pedestrian_walking_confidence = finite("pedestrian_walking_confidence", 0.9);
        playback.virtual_vehicle_fast_speed = positive("virtual_vehicle_fast_speed", 15.0);
        playback.virtual_vehicle_slow_speed = positive("virtual_vehicle_slow_speed", 8.0);
        playback.virtual_vehicle_fast_confidence = finite("virtual_vehicle_fast_confidence", 0.7);
        const auto maximum_virtual_objects = declare_parameter<std::int64_t>("maximum_virtual_objects", 64);
        if (maximum_virtual_objects <= 0) {throw std::runtime_error("maximum_virtual_objects must be positive");}
        playback.maximum_virtual_objects = static_cast<std::size_t>(maximum_virtual_objects);
        const bool trim_zero_padding = declare_parameter<bool>("trim_zero_padding", true);
        m_playback = std::make_unique<ScenarioPlayback>(
            ScenarioData::load(scenario_file, playback.prediction_horizon, trim_zero_padding), playback);
        m_time_step = positive("simulation_step", m_playback->data().time_step);
        const auto real_time_factor = positive("real_time_factor", 1.0);
        m_simulation_time_stamps = declare_parameter<bool>("simulation_time_stamps", false);
        m_use_internal_controller = declare_parameter<bool>("use_internal_controller", true);
        m_use_scenario_speed = declare_parameter<bool>("use_scenario_speed_when_uncommanded", true);
        m_stop_at_end = declare_parameter<bool>("stop_at_scenario_end", true);
        m_command_timeout = positive("command_timeout", 1.0);
        const auto command_mode = declare_parameter<std::string>("command_mode", "speed");
        if (command_mode != "speed" && command_mode != "acceleration")
        {
            throw std::runtime_error("command_mode must be speed or acceleration");
        }
        m_acceleration_commands = command_mode == "acceleration";
        m_wait_for_first_command = declare_parameter<bool>("wait_for_first_control_command", true);
        auto & settings = m_vehicle.settings;
        const auto model = declare_parameter<std::string>("vehicle_model", "kinematic");
        if (model == "dynamic") {settings.model = VehicleModelType::Dynamic;}
        else if (model != "kinematic")
        {
            throw std::runtime_error("vehicle_model must be kinematic or dynamic");
        }
        const auto nonnegative = [&](const std::string & name, double fallback)
        {
            const auto value = finite(name, fallback);
            if (value < 0.0) {throw std::runtime_error(name + " must be nonnegative");}
            return value;
        };
        settings.wheelbase = positive("wheelbase", settings.wheelbase);
        settings.mass = positive("vehicle_mass", settings.mass);
        settings.yaw_inertia = positive("yaw_inertia", settings.yaw_inertia);
        settings.front_axle_distance = positive("front_axle_distance", settings.front_axle_distance);
        settings.front_cornering_stiffness = positive("front_cornering_stiffness", settings.front_cornering_stiffness);
        settings.rear_cornering_stiffness = positive("rear_cornering_stiffness", settings.rear_cornering_stiffness);
        settings.air_density = nonnegative("air_density", settings.air_density);
        settings.drag_coefficient = nonnegative("drag_coefficient", settings.drag_coefficient);
        settings.frontal_area = positive("frontal_area", settings.frontal_area);
        settings.rolling_resistance_coefficient = nonnegative("rolling_resistance_coefficient", settings.rolling_resistance_coefficient);
        settings.viscous_resistance = nonnegative("viscous_resistance", settings.viscous_resistance);
        settings.kinematic_blend_speed = positive("kinematic_blend_speed", settings.kinematic_blend_speed);
        settings.dynamic_blend_speed = positive("dynamic_blend_speed", settings.dynamic_blend_speed);
        settings.integration_step = positive("dynamics_integration_step", settings.integration_step);
        auto & powertrain_settings = m_powertrain->settings;
        powertrain_settings.motor_gear_ratio = positive("motor_gear_ratio", powertrain_settings.motor_gear_ratio);
        powertrain_settings.wheel_gear_ratio = positive("wheel_gear_ratio", powertrain_settings.wheel_gear_ratio);
        powertrain_settings.wheel_radius = positive("wheel_radius", powertrain_settings.wheel_radius);
        powertrain_settings.motor_inertia = positive("motor_inertia", powertrain_settings.motor_inertia);
        powertrain_settings.wheel_inertia = positive("wheel_inertia", powertrain_settings.wheel_inertia);
        powertrain_settings.motor_viscous_friction = nonnegative(
            "motor_viscous_friction", powertrain_settings.motor_viscous_friction);
        powertrain_settings.wheel_viscous_friction = nonnegative(
            "wheel_viscous_friction", powertrain_settings.wheel_viscous_friction);
        powertrain_settings.shaft_stiffness = positive("shaft_stiffness", powertrain_settings.shaft_stiffness);
        powertrain_settings.shaft_damping = nonnegative("shaft_damping", powertrain_settings.shaft_damping);
        powertrain_settings.maximum_motor_torque = positive(
            "maximum_motor_torque", powertrain_settings.maximum_motor_torque);
        powertrain_settings.maximum_regenerative_torque = positive(
            "maximum_regenerative_torque", powertrain_settings.maximum_regenerative_torque);
        powertrain_settings.motor_torque_rate_limit = positive(
            "motor_torque_rate_limit", powertrain_settings.motor_torque_rate_limit);
        powertrain_settings.integration_step = positive(
            "powertrain_integration_step", powertrain_settings.integration_step);
        m_road_pitch = finite("road_pitch", 0.0);
        m_use_road_gradient = declare_parameter<bool>("use_road_gradient", true);
        if (std::abs(m_road_pitch) >= 0.5 * std::acos(-1.0))
        {
            throw std::runtime_error("road_pitch must be between -pi/2 and pi/2 radians");
        }
        settings.maximum_acceleration = positive("maximum_acceleration", settings.maximum_acceleration);
        settings.maximum_deceleration = positive("maximum_deceleration", settings.maximum_deceleration);
        settings.maximum_steering = positive("maximum_steering", settings.maximum_steering);
        if (settings.maximum_steering >= 0.5 * std::acos(-1.0))
        {
            throw std::runtime_error("maximum_steering must be less than pi/2");
        }
        settings.speed_gain = positive("speed_gain", settings.speed_gain);
        settings.lookahead_time = positive("lookahead_time", settings.lookahead_time);
        settings.minimum_lookahead = positive("minimum_lookahead", settings.minimum_lookahead);
        m_acceleration_filter_time_constant = nonnegative(
            "acceleration_command_time_constant", m_acceleration_filter_time_constant);
        m_vehicle.validateSettings();
        auto & state = m_vehicle.state;
        const auto & route = m_playback->data().target_path;
        const auto initial_pose = pathPoseAtDistance(route, nonnegative("initial_displacement", 0.0));
        state.x = finite("initial_x", initial_pose.point.x);
        state.y = finite("initial_y", initial_pose.point.y);
        state.yaw = finite("initial_yaw", initial_pose.heading);
        const auto initial_road = m_playback->roadState(state);
        state.z = initial_road.elevation;
        state.pitch = initial_road.pitch;
        state.speed = finite("initial_speed", 50.0 / 3.6);
        state.acceleration = finite("initial_acceleration", 0.0);
        m_filtered_acceleration = state.acceleration;
        state.lateral_speed = finite("initial_lateral_speed", 0.0);
        state.yaw_rate = finite("initial_yaw_rate", 0.0);
        if (state.speed < 0.0) {throw std::runtime_error("initial_speed cannot be negative");}
        m_powertrain->initialize(state.speed);
        m_pub_scenario = create_publisher<crp_msgs::msg::Scenario>("scenario", 10);
        m_pub_scenario_visualization = create_publisher<crp_msgs::msg::ScenarioVisualization>(
            "/scenario_visualization", 10);
        m_pub_tracking_scenario = create_publisher<crp_msgs::msg::Scenario>("/simulator/tracking_scenario", 1);
        m_pub_ego = create_publisher<crp_msgs::msg::Ego>("ego", 10);
        m_pub_ego_visualization = create_publisher<crp_msgs::msg::EgoVisualization>("/ego_visualization", 10);
        m_pub_powertrain = create_publisher<crp_msgs::msg::PowertrainTelemetry>(
            "/simulator/powertrain_telemetry", 10);
        m_subscription = create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 1,
            [this](autoware_control_msgs::msg::Control::ConstSharedPtr command)
            {
                if (!std::isfinite(m_acceleration_commands ? command->longitudinal.acceleration :
                    command->longitudinal.velocity) ||
                    !std::isfinite(command->lateral.steering_tire_angle))
                {
                    RCLCPP_WARN(get_logger(), "Ignoring non-finite control command");
                    return;
                }
                m_command_speed = std::max(0.0, static_cast<double>(command->longitudinal.velocity));
                m_command_acceleration = command->longitudinal.acceleration;
                m_command_steering = command->lateral.steering_tire_angle;
                m_last_command = std::chrono::steady_clock::now();
                m_received_command = true;
            });
        m_timer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(m_time_step / real_time_factor)), [this]() {run();});
        RCLCPP_INFO(get_logger(), "Loaded %s: %zu lanes, %zu objects, %.2f seconds",
            scenario_file.c_str(), m_playback->data().lanes.size(),
            m_playback->data().actors.size(), m_playback->data().duration());
        RCLCPP_INFO(get_logger(), "Vehicle model: %s", model.c_str());
    }

private:
    void run()
    {
        if (m_started && !(m_acceleration_commands && m_wait_for_first_command && !m_received_command))
        {
            const double step = m_stop_at_end ?
                std::min(m_time_step, m_playback->data().duration() - m_elapsed) : m_time_step;
            double target_speed = m_use_scenario_speed ? m_playback->speedLimit(m_vehicle.state) : 0.0;
            bool command_fresh = false;
            if (m_received_command)
            {
                const auto age = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - m_last_command).count();
                command_fresh = age <= m_command_timeout;
                target_speed = command_fresh ? m_command_speed : 0.0;
            }
            const double steering = m_use_internal_controller ?
                m_vehicle.followPath(m_playback->data().target_path,
                    m_playback->nearestTarget(m_vehicle.state)) : m_command_steering;
            if (step > 0.0)
            {
                const double grade = m_use_road_gradient && m_playback->data().has_elevation ?
                    m_vehicle.state.pitch : m_road_pitch;
                if (m_acceleration_commands)
                {
                    const double acceleration = command_fresh ?
                        m_command_acceleration : -m_vehicle.settings.maximum_deceleration;
                    integratePowertrain(acceleration, steering, step, grade);
                }
                else
                {
                    integratePowertrain(m_vehicle.accelerationTarget(target_speed),
                        steering, step, grade, target_speed);
                }
                const auto road = m_playback->roadState(m_vehicle.state);
                m_vehicle.state.z = road.elevation;
                m_vehicle.state.pitch = road.pitch;
            }
            m_elapsed += step;
        }
        m_started = true;
        const auto stamp = m_simulation_time_stamps ?
            rclcpp::Time(static_cast<std::int64_t>(std::llround(m_elapsed * 1e9)), RCL_ROS_TIME) : now();
        auto scenario = m_playback->message(m_vehicle.state, m_elapsed);
        scenario.header.stamp = stamp;
        scenario.local_moving_objects.header.stamp = stamp;
        scenario.local_obstacles.header.stamp = stamp;
        for (auto & path : scenario.paths)
        {
            path.header.stamp = stamp;
            path.path.header.stamp = stamp;
            path.traffic_rules.header.stamp = stamp;
        }
        crp_msgs::msg::Scenario tracking_scenario;
        tracking_scenario.header = scenario.header;
        tracking_scenario.local_moving_objects = scenario.local_moving_objects;
        tracking_scenario.local_obstacles = scenario.local_obstacles;
        tracking_scenario.virtual_object_ids = scenario.virtual_object_ids;
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
            tracking_scenario.paths.push_back(std::move(target));
        }
        m_pub_tracking_scenario->publish(tracking_scenario);
        crp_msgs::msg::Ego ego;
        ego.header.stamp = stamp;
        ego.header.frame_id = "map_local";
        ego.pose.pose.position.x = m_vehicle.state.x;
        ego.pose.pose.position.y = m_vehicle.state.y;
        ego.pose.pose.position.z = m_vehicle.state.z;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, -m_vehicle.state.pitch, m_vehicle.state.yaw);
        ego.pose.pose.orientation.x = orientation.x(); ego.pose.pose.orientation.y = orientation.y();
        ego.pose.pose.orientation.z = orientation.z(); ego.pose.pose.orientation.w = orientation.w();
        ego.twist.twist.linear.x = m_vehicle.state.speed;
        ego.twist.twist.linear.y = m_vehicle.state.lateral_speed;
        ego.twist.twist.angular.z = m_vehicle.state.yaw_rate;
        ego.accel.accel.linear.x = m_vehicle.state.acceleration;
        ego.accel.accel.linear.y = m_vehicle.state.lateral_acceleration;
        ego.accel.accel.angular.z = m_vehicle.state.yaw_acceleration;
        ego.orientation = static_cast<float>(m_vehicle.state.yaw);
        ego.estimated_orientation = ego.orientation;
        ego.tire_angle_front = static_cast<float>(m_vehicle.state.steering);
        m_pub_ego->publish(ego);
        crp_msgs::msg::PowertrainTelemetry powertrain;
        powertrain.header = ego.header;
        powertrain.motor_speed_rad_s = m_powertrain->state.motor_speed;
        powertrain.motor_torque_nm = m_powertrain->state.motor_torque;
        powertrain.efficiency = m_powertrain->state.efficiency;
        powertrain.electrical_power_w = m_powertrain->state.electrical_power;
        powertrain.energy_consumed_j = m_powertrain->state.energy_consumed;
        powertrain.energy_regenerated_j = m_powertrain->state.energy_regenerated;
        m_pub_powertrain->publish(powertrain);
        crp_msgs::msg::EgoVisualization ego_visualization;
        ego_visualization.header = ego.header;
        ego_visualization.ego = ego;
        ego_visualization.powertrain_telemetry = powertrain;
        m_pub_ego_visualization->publish(ego_visualization);
        m_pub_scenario->publish(scenario);
        crp_msgs::msg::ScenarioVisualization scenario_visualization;
        scenario_visualization.header = scenario.header;
        scenario_visualization.scenario = scenario;
        m_pub_scenario_visualization->publish(scenario_visualization);
        if (m_stop_at_end && m_elapsed >= m_playback->data().duration() - 1e-9)
        {
            m_timer->cancel();
            RCLCPP_INFO(get_logger(), "Scenario finished at %.2f seconds", m_elapsed);
        }
    }

    void integratePowertrain(double acceleration_target, double steering, double duration,
        double grade, double target_speed = 0.0)
    {
        const double integration_step = std::min(m_vehicle.settings.integration_step,
            m_powertrain->settings.integration_step);
        const auto count = static_cast<std::size_t>(std::ceil(duration / integration_step));
        if (count == 0 || count > 100000)
        {
            throw std::runtime_error("Powertrain update requires too many substeps");
        }
        const double step = duration / static_cast<double>(count);
        const double initial_speed = m_vehicle.state.speed;
        const double initial_yaw_rate = m_vehicle.state.yaw_rate;
        double lateral_acceleration = 0.0;
        for (std::size_t iteration = 0; iteration < count; ++iteration)
        {
            const double requested = m_acceleration_commands ? acceleration_target :
                m_vehicle.accelerationTarget(target_speed);
            const double filter_fraction = m_acceleration_filter_time_constant > 0.0 ?
                -std::expm1(-step / m_acceleration_filter_time_constant) : 1.0;
            m_filtered_acceleration += filter_fraction * (requested - m_filtered_acceleration);
            const double wheel_force = m_powertrain->step(m_filtered_acceleration, m_vehicle.state.speed,
                step, m_vehicle.settings);
            m_vehicle.stepWheelForce(wheel_force, steering, step, grade);
            lateral_acceleration += m_vehicle.state.lateral_acceleration * step;
        }
        m_vehicle.state.acceleration = (m_vehicle.state.speed - initial_speed) / duration;
        m_vehicle.state.lateral_acceleration = lateral_acceleration / duration;
        m_vehicle.state.yaw_acceleration = (m_vehicle.state.yaw_rate - initial_yaw_rate) / duration;
        m_powertrain->state.vehicle_speed = m_vehicle.state.speed;
    }

    VehicleModel m_vehicle;
    std::unique_ptr<PowertrainModel> m_powertrain;
    std::unique_ptr<ScenarioPlayback> m_playback;
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_pub_scenario;
    rclcpp::Publisher<crp_msgs::msg::ScenarioVisualization>::SharedPtr m_pub_scenario_visualization;
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_pub_tracking_scenario;
    rclcpp::Publisher<crp_msgs::msg::Ego>::SharedPtr m_pub_ego;
    rclcpp::Publisher<crp_msgs::msg::EgoVisualization>::SharedPtr m_pub_ego_visualization;
    rclcpp::Publisher<crp_msgs::msg::PowertrainTelemetry>::SharedPtr m_pub_powertrain;
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr m_subscription;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::chrono::steady_clock::time_point m_last_command;
    double m_time_step{0.05};
    double m_elapsed{0.0};
    double m_command_speed{0.0};
    double m_command_acceleration{0.0};
    double m_command_steering{0.0};
    double m_command_timeout{1.0};
    double m_road_pitch{0.0};
    double m_acceleration_filter_time_constant{0.05};
    double m_filtered_acceleration{0.0};
    bool m_received_command{false};
    bool m_started{false};
    bool m_use_internal_controller{true};
    bool m_use_scenario_speed{true};
    bool m_stop_at_end{true};
    bool m_use_road_gradient{true};
    bool m_acceleration_commands{false};
    bool m_wait_for_first_command{true};
    bool m_simulation_time_stamps{false};
};

}
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<crp::apl::Simulator>());
    }
    catch (const std::exception & error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("simulator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}