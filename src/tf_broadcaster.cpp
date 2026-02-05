#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TODO: change include to your custom message
#include <crp_msgs/msg/ego.hpp>

class EgoTfBroadcaster : public rclcpp::Node
{
public:
  EgoTfBroadcaster()
  : Node("ego_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = this->create_subscription<crp_msgs::msg::Ego>(
      "ego",
      rclcpp::QoS(10),
      std::bind(&EgoTfBroadcaster::callback, this, std::placeholders::_1));
  }

private:
  void callback(const crp_msgs::msg::Ego::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Timestamp: use your message stamp if it exists, otherwise use node time.
    // If your msg has header.stamp, prefer that.
    // t.header.stamp = msg->header.stamp;
    t.header.stamp = this->get_clock()->now();

    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    // ----------------------------
    // TODO: Adapt these field names
    // Example assumes msg->pose.position.{x,y,z} and msg->pose.orientation.{x,y,z,w}
    // ----------------------------
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
