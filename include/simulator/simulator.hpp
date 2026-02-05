#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP


#include <iostream>
#include <string>
#include <cstdlib>
#include <ctime>

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

struct MovingObject
{
    double x;
    double y;
    double vx;
    double ax;
    double theta;
    uint type; // 1: small car, 2: truck
};

class Simulator : public rclcpp::Node
{
public:
    Simulator();

private:
    void ctrlCmdCallback(const autoware_control_msgs::msg::Control::SharedPtr msg);
    
    void run();

    float dT = 0.02f; // model runs in 20ms
    double runningTimer = 0.0f;

    // member methods
    void vehicleModel();
    double vehicleLongController();
    void vehicleLatController();
    void generateOncomingObjects();
    void generateFollowedObject();  

    void mapEgo();
    void mapScenario();
    Quaternion yawToQuaternion(double);

    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr m_sub_ctrlCmd_;
    
    // new setup 
    rclcpp::Publisher<crp_msgs::msg::Scenario>::SharedPtr m_pub_scenario_;
    rclcpp::Publisher<crp_msgs::msg::Ego>::SharedPtr m_pub_ego_;

    // new messages
    crp_msgs::msg::Scenario m_scenarioMsg;
    crp_msgs::msg::Ego m_egoMsg;

    autoware_control_msgs::msg::Control m_ctrlCmdMsg;

    rclcpp::TimerBase::SharedPtr m_timer_;

    bool m_initialized = false;

    // target values coming from the vehicle
    double m_vehicleSpeedTarget{15.0f};
    double m_targetSteeringAngle{0.0f};

    // ego vehicle states
    double m_axEgo{0.0f}; // in m/s^2
    double m_vxEgo{0.0f}; // in m/s
    double m_xEgo{0.0f}; // in m
    double m_yEgo{0.0f}; // in m
    double m_thetaEgo{0.0f}; // in rad
    double m_yawRate{0.0f}; // in rad/s
    double m_speedInit{20.0f}; // in mps

    double p_axleDistance{2.7};

    // static environmental parameters
    double m_scenarioDistance{1000.0f};
    double m_dx = {0.5f};
    double m_coefficientsEgoLaneCenterline[4] = {0, 0.005, -0.001, 0.000001};
    double m_coefficientsLeftLaneCenterline[4] = {3.9, 0.005, -0.001, 0.000001};

    // vehicle controller parameters
    bool m_useInternalController = true;
    double m_errorIntegral{0.0f};
    double p_lookAheadTime{1.5};
    
    // dynamic environmental parameters
    double p_objectSeparationTime{2.5};
    double m_oncomingObjectGenerationTimer = {0.0f};
    double m_followedObjectGenerationTimer = {0.0f};
    unsigned int p_numberOfFollowedObjects{2U};
    MovingObject initialFollowedObject;

    std::vector<MovingObject> oncomingObjects;
    std::vector<MovingObject> followedObjects;
};

} // namespace apl
} // namespace crp
#endif // SIMULATOR_HPP
