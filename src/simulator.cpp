#include "simulator/simulator.hpp"


crp::apl::Simulator::Simulator() : Node("simulator")
{
    m_sub_ctrlCmd_   = this->create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 10, std::bind(&Simulator::ctrlCmdCallback, this, std::placeholders::_1));

    m_pub_scenario_ = this->create_publisher<crp_msgs::msg::Scenario>("scenario", 10);
    m_pub_ego_      = this->create_publisher<crp_msgs::msg::Ego>("ego", 10);

    this->declare_parameter<std::vector<double>>("/simulator/initial_ego_state", {0.0, 0.0, 0.0}); // in m/s
    // this->declare_parameter("/test/previewDistance", 100.0f); // in m

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Simulator::run, this));

    RCLCPP_INFO(this->get_logger(), "simulator has been started");
}

void crp::apl::Simulator::ctrlCmdCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{
    m_vehicleSpeedTarget = msg->longitudinal.velocity;

    return;
}

void crp::apl::Simulator::vehicleModel()
{
    m_axEgo = vehicleController();
    m_vxEgo = m_vxEgo + dT*m_axEgo;
    m_xEgo = m_xEgo + dT*m_vxEgo;

    return;
}

double crp::apl::Simulator::vehicleController()
{
    double error = m_vehicleSpeedTarget - m_vxEgo;
    m_errorIntegral = m_errorIntegral + dT*error;

    double P = 1.0;
    double I = 0.1;

    return (P*error + I*m_errorIntegral);
}

void crp::apl::Simulator::mapEgo()
{
    m_egoMsg.header.stamp = this->now();
    m_egoMsg.header.frame_id = "map";
    m_egoMsg.pose.pose.position.x = m_xEgo;
    m_egoMsg.twist.twist.linear.x = m_vxEgo;
    m_egoMsg.accel.accel.linear.x = m_axEgo;

    crp::apl::Quaternion q = yawToQuaternion(m_thetaEgo);
    m_egoMsg.pose.pose.orientation.x = q.x;
    m_egoMsg.pose.pose.orientation.y = q.y;
    m_egoMsg.pose.pose.orientation.z = q.z;
    m_egoMsg.pose.pose.orientation.w = q.w;

    return;
}

void crp::apl::Simulator::mapScenario()
{
    m_scenarioMsg.header.stamp = this->now();
    m_scenarioMsg.header.frame_id = "base_link";
    m_scenarioMsg.paths.clear();
    
    crp_msgs::msg::PathWithTrafficRules pathWithTrafficRules;
    pathWithTrafficRules.header.stamp = this->now();
    pathWithTrafficRules.header.frame_id = "base_link";

    tier4_planning_msgs::msg::PathWithLaneId path;

    double x = m_xEgo;

    while (x < m_scenarioDistance)
    {
        tier4_planning_msgs::msg::PathPointWithLaneId pathPoint;
        geometry_msgs::msg::Point leftBoundPoint;
        geometry_msgs::msg::Point rightBoundPoint;
        double global_x = x;
        double global_y = m_coefficients[0] + m_coefficients[1]*x+m_coefficients[2]*std::pow(x,2)+ m_coefficients[3]*std::pow(x,3);

        pathPoint.point.pose.position.x = (global_x - m_xEgo)*std::cos(m_thetaEgo) + (global_y - m_yEgo)*std::sin(m_thetaEgo);
        pathPoint.point.pose.position.y = -(global_x - m_xEgo)*std::sin(m_thetaEgo) + (global_y - m_yEgo)*std::cos(m_thetaEgo);
        pathPoint.point.pose.position.z = 0.0;

        leftBoundPoint.x = pathPoint.point.pose.position.x;
        leftBoundPoint.y = pathPoint.point.pose.position.y + 1.875;

        rightBoundPoint.x = pathPoint.point.pose.position.x;
        rightBoundPoint.y = pathPoint.point.pose.position.y - 1.875;

        crp::apl::Quaternion q = yawToQuaternion(std::atan(m_coefficients[1] +2*m_coefficients[2]*x + 3*m_coefficients[4]*std::pow(x,2)) - m_thetaEgo);
        pathPoint.point.pose.orientation.x = q.x;
        pathPoint.point.pose.orientation.y = q.y;
        pathPoint.point.pose.orientation.z = q.z;
        pathPoint.point.pose.orientation.w = q.w;

        x = x + m_dx;

        path.points.push_back(pathPoint);
        path.left_bound.push_back(leftBoundPoint);
        path.right_bound.push_back(rightBoundPoint);
    }

    pathWithTrafficRules.path = path;

    m_scenarioMsg.paths.push_back(pathWithTrafficRules);
     
    return;
}

crp::apl::Quaternion crp::apl::Simulator::yawToQuaternion(double yaw_rad)
{
  crp::apl::Quaternion q;

  // roll = pitch = 0
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw_rad * 0.5);
  q.w = std::cos(yaw_rad * 0.5);

  return q;
}

void crp::apl::Simulator::run()
{
    // m_behavior.header.stamp = this->now();
    // m_behavior.target_speed.data = m_maximumSpeedInit;

    if(!m_initialized){
        if (this->has_parameter("/simulator/initial_ego_state")){
            std::vector<double> state = this->get_parameter("/simulator/initial_ego_state").as_double_array();
            m_xEgo = state.at(0);
            m_vxEgo = state.at(1);
            m_axEgo = state.at(2);
        }
        m_initialized = true;
    }

    // update vehicle position
    vehicleModel();

    mapEgo();
    mapScenario();

    m_pub_scenario_->publish(m_scenarioMsg);
    m_pub_ego_->publish(m_egoMsg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<crp::apl::Simulator>());
    rclcpp::shutdown();
    return 0;
}