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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
    declare_parameter("body_id", "none");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "none");
    declare_parameter("wheel_right", "none");
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);

    // if these are not specified, shutdown the node
    if (get_parameter("body_id").as_string() == "none"){
      RCLCPP_ERROR_STREAM(get_logger(), "No body frame specified.");
      rclcpp::shutdown();
    }
    else if (get_parameter("wheel_left").as_string() == "none"){
      RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint name specified.");
      rclcpp::shutdown();
    }
    else if (get_parameter("wheel_right").as_string() == "none"){
      RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint name specified.");
      rclcpp::shutdown();
    }

    const auto bdy_id = get_parameter("body_id").as_string();
    body_id = bdy_id;
    const auto odm_id = get_parameter("odom_id").as_string();
    odom_id = odm_id;
    const auto wheel_l = get_parameter("wheel_left").as_string();
    wheel_left = wheel_l;
    const auto wheel_r = get_parameter("wheel_right").as_string();
    wheel_right = wheel_r;
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto depth = get_parameter("track_width").as_double();

    turtlelib::DiffDrive tbot{depth, radius};
    tbot3 = tbot;


    // initialize publishers and timer
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    js_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&OdomNode::js_callback, this, std::placeholders::_1));
   
    // initialp_service_ =
    //     create_service<>(
    //     "~/initial_pose",
    //     std::bind(&OdomNode::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:

  turtlelib::DiffDrive tbot3{0.0,0.0};
  std::string body_id, odom_id, wheel_left, wheel_right;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void js_callback(const sensor_msgs::msg::JointState & msg)
  {
    turtlelib::Twist2D Vb = tbot3.forward_kin(msg.position[0], msg.position[1]);
    tf2::Quaternion q;
    q.setRPY(0.0,0.0,tbot3.q.theta);
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    
    nav_msgs::msg::Odometry odom{};
    odom.header.frame_id = odom_id;
    odom.header.stamp = get_clock()->now();
    odom.child_frame_id = body_id;
    odom.pose.pose.position.x = tbot3.q.x;
    odom.pose.pose.position.y = tbot3.q.y;
    odom.pose.pose.orientation = quat;

    odom.twist.twist.linear.x = Vb.v.x;
    odom.twist.twist.linear.y = Vb.v.y;
    odom.twist.twist.angular.z = Vb.w;
    odom_publisher_->publish(odom);

    // geometry_msgs::msg::TransformStamped t;
    // t.header.stamp = get_clock()->now();
    // t.header.frame_id = odom_id;
    // t.child_frame_id = body_id;

    // t.transform.translation.x = x;
    // t.transform.translation.y = y;

    // t.transform.rotation.x = 0.0;
    // t.transform.rotation.y = 0.0;
    // t.transform.rotation.z = theta;

    // tf_broadcaster_->sendTransform(t);

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
