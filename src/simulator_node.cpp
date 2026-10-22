#include "simulator/scenario_playback.hpp"

#include <autoware_control_msgs/msg/control.hpp>
#include <crp_msgs/msg/ego.hpp>
#include <rclcpp/rclcpp.hpp>

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
        playback.lane_width = positive("lane_width", playback.lane_width);
        playback.target_path_width = positive("target_path_width", playback.target_path_width);
        playback.crossing_width = positive("crossing_width", playback.crossing_width);
        playback.prediction_horizon = positive("prediction_horizon", playback.prediction_horizon);
        playback.object_distance = positive("object_distance", playback.object_distance);
        const bool trim_zero_padding = declare_parameter<bool>("trim_zero_padding", true);
        m_playback = std::make_unique<ScenarioPlayback>(
            ScenarioData::load(scenario_file, playback.prediction_horizon, trim_zero_padding), playback);
        m_time_step = positive("simulation_step", m_playback->data().time_step);
        const auto real_time_factor = positive("real_time_factor", 1.0);
        m_use_internal_controller = declare_parameter<bool>("use_internal_controller", true);
        m_use_scenario_speed = declare_parameter<bool>("use_scenario_speed_when_uncommanded", true);
        m_stop_at_end = declare_parameter<bool>("stop_at_scenario_end", true);
        m_command_timeout = positive("command_timeout", 1.0);
        auto & settings = m_vehicle.settings;
        settings.wheelbase = positive("wheelbase", settings.wheelbase);
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
        auto & state = m_vehicle.state;
        const auto & route = m_playback->data().target_path;
        state.x = finite("initial_x", route.front().x);
        state.y = finite("initial_y", route.front().y);
        state.yaw = finite("initial_yaw", pathHeading(route, 0));
        state.speed = finite("initial_speed", 0.0);
        state.acceleration = finite("initial_acceleration", 0.0);
        if (state.speed < 0.0) {throw std::runtime_error("initial_speed cannot be negative");}
        m_pub_scenario = create_publisher<crp_msgs::msg::Scenario>("scenario", 10);
        m_pub_ego = create_publisher<crp_msgs::msg::Ego>("ego", 10);
        m_subscription = create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 10,
            [this](autoware_control_msgs::msg::Control::ConstSharedPtr command)
            {
                if (!std::isfinite(command->longitudinal.velocity) ||
                    !std::isfinite(command->lateral.steering_tire_angle))
                {
                    RCLCPP_WARN(get_logger(), "Ignoring non-finite control command");
                    return;
                }
                m_command_speed = std::max(0.0, static_cast<double>(command->longitudinal.velocity));
                m_command_steering = command->lateral.steering_tire_angle;
                m_last_command = std::chrono::steady_clock::now();
                m_received_command = true;
            });
        m_timer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(m_time_step / real_time_factor)), [this]() {run();});
        RCLCPP_INFO(get_logger(), "Loaded %s: %zu lanes, %zu objects, %.2f seconds",
            scenario_file.c_str(), m_playback->data().lanes.size(),
            m_playback->data().actors.size(), m_playback->data().duration());
    }

private:
    void run()
    {
        if (m_started)
        {
            const double step = m_stop_at_end ?
                std::min(m_time_step, m_playback->data().duration() - m_elapsed) : m_time_step;
            double target_speed = m_use_scenario_speed ? m_playback->speedLimit(m_vehicle.state) : 0.0;
            if (m_received_command)
            {
                const auto age = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - m_last_command).count();
                target_speed = age <= m_command_timeout ? m_command_speed : 0.0;
            }
            const double steering = m_use_internal_controller ?
                m_vehicle.followPath(m_playback->data().target_path,
                    m_playback->nearestTarget(m_vehicle.state)) : m_command_steering;
            if (step > 0.0) {m_vehicle.step(target_speed, steering, step);}
            m_elapsed += step;
        }
        m_started = true;
        const auto stamp = now();
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
        crp_msgs::msg::Ego ego;
        ego.header.stamp = stamp;
        ego.header.frame_id = "map";
        ego.pose.pose.position.x = m_vehicle.state.x;
        ego.pose.pose.position.y = m_vehicle.state.y;
        ego.pose.pose.orientation.z = std::sin(0.5 * m_vehicle.state.yaw);
        ego.pose.pose.orientation.w = std::cos(0.5 * m_vehicle.state.yaw);
        ego.twist.twist.linear.x = m_vehicle.state.speed;
        ego.twist.twist.angular.z = m_vehicle.state.yaw_rate;
        ego.accel.accel.linear.x = m_vehicle.state.acceleration;
        ego.orientation = static_cast<float>(m_vehicle.state.yaw);
        ego.estimated_orientation = ego.orientation;
        ego.tire_angle_front = static_cast<float>(m_vehicle.state.steering);
        m_pub_ego->publish(ego);
        m_pub_scenario->publish(scenario);
        if (m_stop_at_end && m_elapsed >= m_playback->data().duration() - 1e-9)
        {
            m_timer->cancel();
            RCLCPP_INFO(get_logger(), "Scenario finished at %.2f seconds", m_elapsed);
        }
    }

    VehicleModel m_vehicle;
    std::unique_ptr<ScenarioPlayback> m_playback;
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_pub_scenario;
    rclcpp::Publisher<crp_msgs::msg::Ego>::SharedPtr m_pub_ego;
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr m_subscription;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::chrono::steady_clock::time_point m_last_command;
    double m_time_step{0.05};
    double m_elapsed{0.0};
    double m_command_speed{0.0};
    double m_command_steering{0.0};
    double m_command_timeout{1.0};
    bool m_received_command{false};
    bool m_started{false};
    bool m_use_internal_controller{true};
    bool m_use_scenario_speed{true};
    bool m_stop_at_end{true};
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