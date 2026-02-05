#include "simulator/simulator.hpp"


crp::apl::Simulator::Simulator() : Node("simulator")
{
    m_sub_ctrlCmd_   = this->create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 10, std::bind(&Simulator::ctrlCmdCallback, this, std::placeholders::_1));

    m_pub_scenario_ = this->create_publisher<crp_msgs::msg::Scenario>("scenario", 10);
    m_pub_ego_      = this->create_publisher<crp_msgs::msg::Ego>("ego", 10);

    this->declare_parameter<std::vector<double>>("/simulator/initial_ego_state", {0.0, 10.0, 0.0}); // in m/s
    this->declare_parameter<std::vector<double>>("/simulator/initial_object_state", {100.0, 8.0, -5.0}); // in m/s
    this->declare_parameter("/simulator/use_internal_controller", true); 
    this->declare_parameter("/simulator/object_disappear_time", 12.0); 

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Simulator::run, this));

    RCLCPP_INFO(this->get_logger(), "simulator has been started");
}

void crp::apl::Simulator::ctrlCmdCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{
    m_vehicleSpeedTarget = msg->longitudinal.velocity;

    if(!m_useInternalController)
    {
        // external steering angle command is used
        m_targetSteeringAngle = msg->lateral.steering_tire_angle ;
    }

    return;
}

void crp::apl::Simulator::vehicleModel()
{
    m_axEgo = vehicleLongController();
    m_vxEgo = m_vxEgo + dT*m_axEgo;

    m_yawRate = std::tan(m_targetSteeringAngle)/p_axleDistance*m_vxEgo;
    m_thetaEgo = m_thetaEgo + m_yawRate*dT;

    m_xEgo = m_xEgo + dT*m_vxEgo*std::cos(m_thetaEgo);
    m_yEgo = m_yEgo + dT*m_vxEgo*std::sin(m_thetaEgo);

    return;
}

double crp::apl::Simulator::vehicleLongController()
{
    double error = m_vehicleSpeedTarget - m_vxEgo;
    m_errorIntegral = m_errorIntegral + dT*error;

    double P = 1.0;
    double I = 0.1;

    return (P*error + I*m_errorIntegral);
}

void crp::apl::Simulator::vehicleLatController()
{
    crp_msgs::msg::PathWithTrafficRules pathWithTrafficRules;
    tier4_planning_msgs::msg::PathWithLaneId path;

    if (m_scenarioMsg.paths.size() > 0){
        path = m_scenarioMsg.paths.at(0).path;
        for (long unsigned int np=0; np<path.points.size(); np++)
        {
            tier4_planning_msgs::msg::PathPointWithLaneId pathPoint;
            pathPoint = path.points.at(np);

            if(std::sqrt(std::pow(pathPoint.point.pose.position.x, 2)+std::pow(pathPoint.point.pose.position.y, 2)) > (p_lookAheadTime*m_vxEgo))
            {
                if (std::abs(pathPoint.point.pose.position.y) < 0.01)
                {
                    m_targetSteeringAngle = 0.0;
                }
                else{
                    double r = std::pow((p_lookAheadTime*m_vxEgo), 2) / (2*pathPoint.point.pose.position.y);
                    if (std::abs(r) < 1e-8)
                    {
                        m_targetSteeringAngle = 0.0;
                    }
                    else{
                        m_targetSteeringAngle = std::atan(p_axleDistance / r);
                    }
                }                
                break;
            }
        }
    }  

    return;
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

    m_egoMsg.orientation = m_thetaEgo;
    m_egoMsg.tire_angle_front = m_targetSteeringAngle;

    return;
}

void crp::apl::Simulator::mapScenario()
{
    m_scenarioMsg.header.stamp = this->now();
    m_scenarioMsg.header.frame_id = "base_link";
    m_scenarioMsg.paths.clear();
    m_scenarioMsg.local_moving_objects.objects.clear();
    
    // lanes

    // ego lane
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
        double global_y = m_coefficientsEgoLaneCenterline[0] + 
            m_coefficientsEgoLaneCenterline[1]*x + 
            m_coefficientsEgoLaneCenterline[2]*std::pow(x,2) + 
            m_coefficientsEgoLaneCenterline[3]*std::pow(x,3);

        pathPoint.point.pose.position.x = (global_x - m_xEgo)*std::cos(m_thetaEgo) + (global_y - m_yEgo)*std::sin(m_thetaEgo);
        pathPoint.point.pose.position.y = -(global_x - m_xEgo)*std::sin(m_thetaEgo) + (global_y - m_yEgo)*std::cos(m_thetaEgo);
        pathPoint.point.pose.position.z = 0.0;

        leftBoundPoint.x = pathPoint.point.pose.position.x;
        leftBoundPoint.y = pathPoint.point.pose.position.y + 1.875;

        rightBoundPoint.x = pathPoint.point.pose.position.x;
        rightBoundPoint.y = pathPoint.point.pose.position.y - 1.875;

        crp::apl::Quaternion q = yawToQuaternion(std::atan(m_coefficientsEgoLaneCenterline[1] +2*m_coefficientsEgoLaneCenterline[2]*x + 3*m_coefficientsEgoLaneCenterline[4]*std::pow(x,2)) - m_thetaEgo);
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

    // opposite lane    
    crp_msgs::msg::PathWithTrafficRules opponentPathWithTrafficRules;
    opponentPathWithTrafficRules.header.stamp = this->now();
    opponentPathWithTrafficRules.header.frame_id = "base_link";

    tier4_planning_msgs::msg::PathWithLaneId opponentPath;

    x = m_xEgo;

    while (x < m_scenarioDistance)
    {
        tier4_planning_msgs::msg::PathPointWithLaneId pathPoint;
        geometry_msgs::msg::Point leftBoundPoint;
        geometry_msgs::msg::Point rightBoundPoint;
        double global_x = x;
        double global_y = m_coefficientsLeftLaneCenterline[0] + 
            m_coefficientsLeftLaneCenterline[1]*x + 
            m_coefficientsLeftLaneCenterline[2]*std::pow(x,2) + 
            m_coefficientsLeftLaneCenterline[3]*std::pow(x,3);

        pathPoint.point.pose.position.x = (global_x - m_xEgo)*std::cos(m_thetaEgo) + (global_y - m_yEgo)*std::sin(m_thetaEgo);
        pathPoint.point.pose.position.y = -(global_x - m_xEgo)*std::sin(m_thetaEgo) + (global_y - m_yEgo)*std::cos(m_thetaEgo);
        pathPoint.point.pose.position.z = 0.0;

        leftBoundPoint.x = pathPoint.point.pose.position.x;
        leftBoundPoint.y = pathPoint.point.pose.position.y + 1.875;

        rightBoundPoint.x = pathPoint.point.pose.position.x;
        rightBoundPoint.y = pathPoint.point.pose.position.y - 1.875;

        crp::apl::Quaternion q = yawToQuaternion(std::atan(m_coefficientsLeftLaneCenterline[1] +2*m_coefficientsLeftLaneCenterline[2]*x + 3*m_coefficientsLeftLaneCenterline[4]*std::pow(x,2)) - m_thetaEgo);
        pathPoint.point.pose.orientation.x = q.x;
        pathPoint.point.pose.orientation.y = q.y;
        pathPoint.point.pose.orientation.z = q.z;
        pathPoint.point.pose.orientation.w = q.w;

        x = x + m_dx;

        opponentPath.points.push_back(pathPoint);
        opponentPath.left_bound.push_back(leftBoundPoint);
        opponentPath.right_bound.push_back(rightBoundPoint);
    }

    opponentPathWithTrafficRules.path = opponentPath;

    m_scenarioMsg.paths.push_back(opponentPathWithTrafficRules);

    // oncoming objects
    for (long unsigned int i = 0; i<oncomingObjects.size(); i++)
    {
        // loop through the objects and transform them into the local frame
        // write it to the scenario message
        double global_x = oncomingObjects.at(i).x;
        double global_y = oncomingObjects.at(i).y;
        double global_theta = oncomingObjects.at(i).theta;

        autoware_perception_msgs::msg::PredictedObject object;
        object.kinematics.initial_pose_with_covariance.pose.position.x = (global_x - m_xEgo)*std::cos(m_thetaEgo) + (global_y - m_yEgo)*std::sin(m_thetaEgo);
        object.kinematics.initial_pose_with_covariance.pose.position.y = -(global_x - m_xEgo)*std::sin(m_thetaEgo) + (global_y - m_yEgo)*std::cos(m_thetaEgo);
        crp::apl::Quaternion q = yawToQuaternion(global_theta-m_thetaEgo);
        object.kinematics.initial_pose_with_covariance.pose.orientation.x = q.x;
        object.kinematics.initial_pose_with_covariance.pose.orientation.y = q.y;
        object.kinematics.initial_pose_with_covariance.pose.orientation.z = q.z;
        object.kinematics.initial_pose_with_covariance.pose.orientation.w = q.w;        
        
        autoware_perception_msgs::msg::ObjectClassification objectClassification;
        objectClassification.label = oncomingObjects.at(i).type;
        object.classification.push_back(objectClassification);

        m_scenarioMsg.local_moving_objects.objects.push_back(object);
    }

    // followed objects
    for (long unsigned int i = 0; i<followedObjects.size(); i++)
    {
        // loop through the objects and transform them into the local frame
        // write it to the scenario message
        double global_x = followedObjects.at(i).x;
        double global_y = followedObjects.at(i).y;
        double global_theta = followedObjects.at(i).theta;

        autoware_perception_msgs::msg::PredictedObject object;
        object.kinematics.initial_pose_with_covariance.pose.position.x = (global_x - m_xEgo)*std::cos(m_thetaEgo) + (global_y - m_yEgo)*std::sin(m_thetaEgo);
        object.kinematics.initial_pose_with_covariance.pose.position.y = -(global_x - m_xEgo)*std::sin(m_thetaEgo) + (global_y - m_yEgo)*std::cos(m_thetaEgo);
        crp::apl::Quaternion q = yawToQuaternion(global_theta-m_thetaEgo);
        object.kinematics.initial_pose_with_covariance.pose.orientation.x = q.x;
        object.kinematics.initial_pose_with_covariance.pose.orientation.y = q.y;
        object.kinematics.initial_pose_with_covariance.pose.orientation.z = q.z;
        object.kinematics.initial_pose_with_covariance.pose.orientation.w = q.w;        
        
        autoware_perception_msgs::msg::ObjectClassification objectClassification;
        objectClassification.label = followedObjects.at(i).type;
        object.classification.push_back(objectClassification);

        m_scenarioMsg.local_moving_objects.objects.push_back(object);
    }
     
    return;
}

void crp::apl::Simulator::generateFollowedObject()
{
        // generating all objects first, then just moving them along the route
        if (followedObjects.size()==0 && m_initialized)
        {
            for (int i=0; i<p_numberOfFollowedObjects; i++)
            {
                // no object yet, generate them first
                // initial followed object comes first
                if (i==0){
                    initialFollowedObject.y = m_coefficientsEgoLaneCenterline[0] + 
                        m_coefficientsEgoLaneCenterline[1]*initialFollowedObject.x + 
                        m_coefficientsEgoLaneCenterline[2]*std::pow(initialFollowedObject.x,2) + 
                        m_coefficientsEgoLaneCenterline[3]*std::pow(initialFollowedObject.x,3);

                    initialFollowedObject.theta = std::atan(m_coefficientsEgoLaneCenterline[1] + 
                        m_coefficientsEgoLaneCenterline[2]*2*initialFollowedObject.x + 
                        m_coefficientsEgoLaneCenterline[3]*3*std::pow(initialFollowedObject.x,2));

                    initialFollowedObject.type = 1U;

                    followedObjects.push_back(initialFollowedObject);
                }
                else{
                    MovingObject nextFollowedObject;
                    double dx = followedObjects.at(i-1).vx*p_objectSeparationTime;
                    nextFollowedObject.x = followedObjects.at(i-1).x + dx;
                    nextFollowedObject.y = m_coefficientsEgoLaneCenterline[0] + 
                        m_coefficientsEgoLaneCenterline[1]*nextFollowedObject.x + 
                        m_coefficientsEgoLaneCenterline[2]*std::pow(nextFollowedObject.x,2) + 
                        m_coefficientsEgoLaneCenterline[3]*std::pow(nextFollowedObject.x,3);

                    nextFollowedObject.theta = std::atan(m_coefficientsEgoLaneCenterline[1] + 
                        m_coefficientsEgoLaneCenterline[2]*2*nextFollowedObject.x + 
                        m_coefficientsEgoLaneCenterline[3]*3*std::pow(nextFollowedObject.x,2));

                    nextFollowedObject.vx = initialFollowedObject.vx;
                    nextFollowedObject.ax = 0.0;

                    nextFollowedObject.type = 1U;

                    followedObjects.push_back(nextFollowedObject);
                }   
            } 
        }
        else if(m_initialized){
            // objects exist, only need to shift them along the route
            for (int i=0; i<followedObjects.size(); i++)
            {
                followedObjects.at(i).vx = followedObjects.at(i).vx + followedObjects.at(i).ax*dT;
                if (followedObjects.at(i).vx <=0){
                    followedObjects.at(i).vx = 0.0f;
                    followedObjects.at(i).ax = 0.0f;
                }
                else{
                    followedObjects.at(i).x = followedObjects.at(i).x + followedObjects.at(i).vx*dT;
                    followedObjects.at(i).y = m_coefficientsEgoLaneCenterline[0] + 
                            m_coefficientsEgoLaneCenterline[1]*followedObjects.at(i).x + 
                            m_coefficientsEgoLaneCenterline[2]*std::pow(followedObjects.at(i).x,2) + 
                            m_coefficientsEgoLaneCenterline[3]*std::pow(followedObjects.at(i).x,3);
                    
                    followedObjects.at(i).theta = std::atan(m_coefficientsEgoLaneCenterline[1] + 
                            m_coefficientsEgoLaneCenterline[2]*2*followedObjects.at(i).x + 
                            m_coefficientsEgoLaneCenterline[3]*3*std::pow(followedObjects.at(i).x,2));
                    }                
            }
        }
    return;
}

void crp::apl::Simulator::generateOncomingObjects()
{
    m_oncomingObjectGenerationTimer = m_oncomingObjectGenerationTimer + dT;

    // loop through objects and adjust their position
    for (long unsigned int i = 0; i<oncomingObjects.size(); i++)
    {
        oncomingObjects.at(i).x = oncomingObjects.at(i).x + oncomingObjects.at(i).vx * dT;
        oncomingObjects.at(i).y = m_coefficientsLeftLaneCenterline[0] + 
            m_coefficientsLeftLaneCenterline[1]*oncomingObjects.at(i).x + 
            m_coefficientsLeftLaneCenterline[2]*std::pow(oncomingObjects.at(i).x,2) + 
            m_coefficientsLeftLaneCenterline[3]*std::pow(oncomingObjects.at(i).x,3);
        oncomingObjects.at(i).theta = std::atan(m_coefficientsLeftLaneCenterline[1] + 
            m_coefficientsLeftLaneCenterline[2]*2*oncomingObjects.at(i).x + 
            m_coefficientsLeftLaneCenterline[3]*3*std::pow(oncomingObjects.at(i).x,2));

        if (oncomingObjects.at(i).x < -m_scenarioDistance)
        {
            // remove the object from the vector as it passes
            oncomingObjects.erase(oncomingObjects.begin());
        }
    }

    // add new object in the left lane only!
    std::srand(std::time(nullptr));   // seed once
    int r = std::rand();              // random int
    if (abs(r)%7==0 && m_oncomingObjectGenerationTimer > p_objectSeparationTime)
    {
        m_oncomingObjectGenerationTimer = 0.0;
        // new object shall be added to the vector
        MovingObject newObject;
        newObject.x = m_scenarioDistance;
        newObject.y = m_coefficientsLeftLaneCenterline[0] + 
            m_coefficientsLeftLaneCenterline[1]*newObject.x + 
            m_coefficientsLeftLaneCenterline[2]*std::pow(newObject.x,2) + 
            m_coefficientsLeftLaneCenterline[3]*std::pow(newObject.x,3);
        newObject.theta = std::atan(m_coefficientsLeftLaneCenterline[1] + 
            m_coefficientsLeftLaneCenterline[2]*2*newObject.x + 
            m_coefficientsLeftLaneCenterline[3]*3*std::pow(newObject.x,2));

        if (abs(r)%3==0)
        {
            // truck
            newObject.vx = -20.0;
            newObject.type = 2U;
        }
        else{
            // small car
            newObject.vx = -27.0;
            newObject.type = 1U;
        }
        oncomingObjects.push_back(newObject);
    }
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
    runningTimer = runningTimer + dT;

    if(!m_initialized){
        if (this->has_parameter("/simulator/initial_ego_state")){
            std::vector<double> state = this->get_parameter("/simulator/initial_ego_state").as_double_array();
            m_xEgo = state.at(0);
            m_vxEgo = state.at(1);
            m_axEgo = state.at(2);
        }
        if (this->has_parameter("/simulator/initial_object_state")){
            std::vector<double> state = this->get_parameter("/simulator/initial_object_state").as_double_array();
            initialFollowedObject.x = state.at(0);
            initialFollowedObject.vx = state.at(1);
            initialFollowedObject.ax = state.at(2);
        }
        m_initialized = true;
    }

    m_useInternalController = this->get_parameter("/simulator/use_internal_controller").as_bool();

    if(m_useInternalController)
    {
        // Long controller runs all the time as the ctrlCmd contains only the velocity level target!
        vehicleLatController();
    }

    // first vehicle can disappear
    if (runningTimer >= this->get_parameter("/simulator/object_disappear_time").as_double())
    {
        followedObjects.erase(followedObjects.begin());
    }

    // update vehicle position
    vehicleModel();

    // generate objects
    generateOncomingObjects();
    generateFollowedObject();

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