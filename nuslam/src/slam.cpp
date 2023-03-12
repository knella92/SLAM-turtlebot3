/// \slam.cpp
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
#include "turtlelib/ekf.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/path.hpp"


class SlamNode : public rclcpp::Node
{
public:
  SlamNode()
  : Node("slam")
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
    declare_parameter("process_covariance", 0.001);

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
    process_covariance = get_parameter("process_covariance").as_double();

    turtlelib::DiffDrive tbot{depth, radius};
    tbot3 = tbot;
    turtlelib::EKF ekf{tbot3.q, max_obstacles, process_covariance, 0.001};
    extended_kalman = ekf;

    green_path_msg.header.frame_id = odom_id;
    blue_path_msg.header.frame_id = odom_id;


    // initialize publisher, subscriber, and service
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    blue_path_ = create_publisher<nav_msgs::msg::Path>("blue/path", 10);
    green_path_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    slam_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/slam_map", 10);
    js_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &SlamNode::js_callback, this,
        std::placeholders::_1));
    fake_sensor_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "nusim/fake_sensor", 10, std::bind(
        &SlamNode::sensor_callback, this,
        std::placeholders::_1));
    initialp_service_ =
      create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(&SlamNode::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_green_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  turtlelib::DiffDrive tbot3{0.0, 0.0};
  int index{0};
  double process_covariance{};
  int max_obstacles{6};
  turtlelib::EKF extended_kalman{tbot3.q, 1, 0.0, 0.0};
  bool obstacles_initialized{false};
  nav_msgs::msg::Path green_path_msg;
  nav_msgs::msg::Path blue_path_msg;
  std::string body_id, odom_id, wheel_left, wheel_right;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blue_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_green_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initialp_service_;

  void js_callback(const sensor_msgs::msg::JointState & msg)
  {
    turtlelib::Twist2D Vb = tbot3.forward_kin(
      msg.position[0] - tbot3.phi_l,
      msg.position[1] - tbot3.phi_r);

    //updating wheel pose with new wheel angles
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
    

    tf2::Quaternion q_green;
    q_green.setRPY(0.0, 0.0, extended_kalman.zeta_est(0));
    geometry_msgs::msg::Quaternion quat_green = tf2::toMsg(q_green);
    geometry_msgs::msg::TransformStamped t_green;
    t_green.header.stamp = get_clock()->now();
    t_green.header.frame_id = odom_id;
    t_green.child_frame_id = "green/base_footprint";
    t_green.transform.translation.x = extended_kalman.zeta_est(1);
    t_green.transform.translation.y = extended_kalman.zeta_est(2);
    t_green.transform.rotation = quat_green;
    tf_green_broadcaster_->sendTransform(t_green);
    
    if(index == 100)
    {
      green_path_msg.header.stamp = get_clock()->now();
      blue_path_msg.header.stamp = get_clock()->now();
      blue_path_->publish(blue_path_msg);
      green_path_->publish(green_path_msg);
      index = 0;
    }
    else{
      green_path_msg.poses.push_back(publish_path(quat_green,"green/base_footprint", extended_kalman.zeta_est(1), extended_kalman.zeta_est(2)));
      blue_path_msg.poses.push_back(publish_path(quat, "blue/base_footprint", tbot3.q.x, tbot3.q.y));
      index++;
    }
     
    
  }

  void initial_pose(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {

    tbot3.q.x = request->x;
    tbot3.q.y = request->y;
    tbot3.q.theta = request->theta;

  }

  geometry_msgs::msg::PoseStamped publish_path(geometry_msgs::msg::Quaternion quat, std::string frame_id, double x, double y)
  {
    
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = get_clock()->now();
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.orientation = quat;
    return pose_msg;
  }

  void sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    extended_kalman.prediction(tbot3.q);

    for(int i{0}; i < 2; i++)
    {
      if(msg.markers[i].action == visualization_msgs::msg::Marker::ADD)
      {

        if(extended_kalman.izd.at(i) == false)
        {
          extended_kalman.initialization(msg.markers[i].id, msg.markers[i].pose.position.x, msg.markers[i].pose.position.y);
          extended_kalman.izd.at(i) = true;
          obstacles_initialized = true;
        }
        
        if(extended_kalman.izd.at(i) == true)
        {
          extended_kalman.correction(msg.markers[i].id, msg.markers[i].pose.position.x, msg.markers[i].pose.position.y);
        }
      }
      
    }
    if(obstacles_initialized)
    {
      slam_publisher_->publish(add_obstacles(msg));
    }
    
  }

  visualization_msgs::msg::MarkerArray add_obstacles(visualization_msgs::msg::MarkerArray msg)
  {
    visualization_msgs::msg::MarkerArray all_obst{};
    rclcpp::Time stamp = get_clock()->now();
    for (int i = 0; i < max_obstacles; ++i)
    {
      if(extended_kalman.izd.at(i) == true)
      {
        visualization_msgs::msg::Marker obst;
        obst.header.frame_id = "nusim/world";
        obst.header.stamp = stamp;
        obst.type = visualization_msgs::msg::Marker::CYLINDER;
        obst.scale.x = msg.markers[i].scale.x;
        obst.scale.y = msg.markers[i].scale.y;
        obst.scale.z = msg.markers[i].scale.z;
        obst.color.a = 1.0;
        obst.color.g = 1.0;
        obst.id = i;
        obst.pose.position.x = extended_kalman.zeta_est(3+(2*i));
        obst.pose.position.y = extended_kalman.zeta_est(4+(2*i));  
        obst.action = visualization_msgs::msg::Marker::ADD;
        // RCLCPP_INFO_STREAM(get_logger(), "green bot y position: " << extended_kalman.zeta_est(2));

        all_obst.markers.push_back(obst);
      }
    }

    return all_obst;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}
