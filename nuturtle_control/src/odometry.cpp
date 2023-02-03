/// \turtle_control.cpp
/// \brief Defines and launche nusim node with appropriate publishers and services
///
/// PARAMETERS:

/// PUBLISHES:

/// SUBSCRIBES:

/// SERVERS:

/// CLIENTS:


#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odometry")
  {

    //declare initial parameters
    declare_parameter("body_id", 0.0);
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "red");
    declare_parameter("wheel_right", "red");

    // // if these are not specified, shutdown the node
    // if (get_parameter("body_id").as_double() == 0.0){
    //   RCLCPP_ERROR_STREAM(get_logger(), "No body frame specified.");
    //   rclcpp::shutdown();
    // }
    // if (get_parameter("wheel_left").as_double() == 0.0){
    //   RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint name specified.");
    //   rclcpp::shutdown();
    // }
    // if (get_parameter("wheel_right").as_double() == 0.0){
    //   RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint name specified.");
    //   rclcpp::shutdown();
    // }

    // const auto body_id = get_parameter("body_id").as_string();
    // const auto odom_id = get_parameter("odom_id").as_string();
    // const auto wheel_left = get_parameter("wheel_left").as_string();
    // const auto wheel_right = get_parameter("wheel_right").as_string();


    // initialize publishers and timer
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    //js_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&OdomNode::js_callback, this));
   
    // initialp_service_ =
    //     create_service<>(
    //     "~/initial_pose",
    //     std::bind(&OdomNode::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;



  void js_callback()
  {

    //wheel_publisher_->publish(message);

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
