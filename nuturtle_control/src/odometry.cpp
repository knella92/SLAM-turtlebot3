/// \odometry.cpp
/// \brief Defines and launches odometry node to update and publish/broadcast joint positions and velocities
///
/// PARAMETERS:
///     wheel_radius (double): Radius of the turtlebot's wheels
///     track_width (double): Distance between the turtlebot's wheels
///     body_id (string): Name of odometry frame's child frame (base_footprint)
///     odom_id (string): Name of odometry frame
///     wheel_left (string): Name of left wheel joint
///     wheel_right (string): Name of right wheel joint
/// PUBLISHES:
///     (/odom) (Odometry): Body configuration and twist
/// SUBSCRIBES:
///     (/joint_states) (JointState): Joint position and velocity messages
/// SERVERS:
///     (/initial_pose) (InitialPose): Input for a full reset of odometry to begin at desired position and orientation
/// CLIENTS:
///     none


#include <functional>
#include <memory>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/path.hpp"


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
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);

    // if these are not specified, shutdown the node
    if (get_parameter("body_id").as_string() == "none") {
      RCLCPP_ERROR_STREAM(get_logger(), "No body frame specified.");
      rclcpp::shutdown();
    } else if (get_parameter("wheel_left").as_string() == "none") {
      RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint name specified.");
      rclcpp::shutdown();
    } else if (get_parameter("wheel_right").as_string() == "none") {
      RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint name specified.");
      rclcpp::shutdown();
    }

    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto depth = get_parameter("track_width").as_double();

    turtlelib::DiffDrive tbot{depth, radius};
    tbot3 = tbot;


    // initialize publisher, subscriber, and service
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
    js_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &OdomNode::js_callback, this,
        std::placeholders::_1));
    initialp_service_ =
      create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(&OdomNode::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  turtlelib::DiffDrive tbot3{0.0, 0.0};
  std::string body_id, odom_id, wheel_left, wheel_right;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initialp_service_;

  void js_callback(const sensor_msgs::msg::JointState & msg)
  {
    turtlelib::Twist2D Vb = tbot3.forward_kin(
      msg.position[0] - tbot3.phi_l,
      msg.position[1] - tbot3.phi_r);
    tbot3.phi_l = msg.position[0];
    tbot3.phi_r = msg.position[1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, tbot3.q.theta);
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

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;

    t.transform.translation.x = tbot3.q.x;
    t.transform.translation.y = tbot3.q.y;
    t.transform.rotation = quat;

    tf_broadcaster_->sendTransform(t);

    publish_path(quat);

  }

  void initial_pose(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {

    tbot3.q.x = request->x;
    tbot3.q.y = request->y;
    tbot3.q.theta = request->theta;

  }

  void publish_path(geometry_msgs::msg::Quaternion quat)
  {
    auto path_msg = nav_msgs::msg::Path();
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = get_clock()->now();
    pose_msg.header.frame_id = "nusim/world";
    pose_msg.pose.position.x = tbot3.q.x;
    pose_msg.pose.position.y = tbot3.q.y;
    pose_msg.pose.orientation = quat;
    path_msg.poses.push_back(pose_msg);
    path_msg.header.stamp = pose_msg.header.stamp;
    path_msg.header.frame_id = pose_msg.header.frame_id;
    path_publisher_->publish(path_msg);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
