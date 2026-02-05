#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP


#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/scenario.hpp>

#include <autoware_control_msgs/msg/control.hpp>

namespace crp
{
namespace apl
{

struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

class Simulator : public rclcpp::Node
{
public:
    Simulator();

private:
    void ctrlCmdCallback(const autoware_control_msgs::msg::Control::SharedPtr msg);
    
    void run();

    // member methods
    void vehicleModel();
    double vehicleController();

    float dT = 0.02f; // model runs in 20ms

    void mapEgo();
    void mapScenario();

    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr m_sub_ctrlCmd_;
    
    // new setup 
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_pub_scenario_;
    rclcpp::Publisher<crp_msgs::msg::Ego>::SharedPtr m_pub_ego_;

    // new messages
    crp_msgs::msg::Scenario m_scenarioMsg;
    crp_msgs::msg::Ego m_egoMsg;

    autoware_control_msgs::msg::Control m_ctrlCmdMsg;

    rclcpp::TimerBase::SharedPtr m_timer_;

    // target values coming from the vehicle
    double m_vehicleSpeedTarget{15.0f};
    double m_axEgo{0.0f}; // in m/s^2
    double m_vxEgo{0.0f}; // in m/s
    double m_xEgo{0.0f}; // in m
    double m_yEgo{0.0f}; // in m
    double m_thetaEgo{0.0f}; // in rad

    double m_scenarioDistance{500.0f};
    double m_dx = {0.5f};
    double m_coefficients[4] = {0, 0.01, -0.001, 0.000001};

    double m_speedInit{20.0f}; // in mps
    double m_errorIntegral{0.0f};

    bool m_initialized = false;

    Quaternion yawToQuaternion(double);
};

} // namespace apl
} // namespace crp
#endif // SIMULATOR_HPP
