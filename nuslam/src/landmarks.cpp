/// \landmarks.cpp
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

#include <chrono>
#include <functional>
#include <memory>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/ml.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class LandmarksNode : public rclcpp::Node
{
public:
  LandmarksNode()
  : Node("landmarks")
  {

    //declare initial parameters
    // declare_parameter("body_id", "none");
    // declare_parameter("odom_id", "odom");
    // declare_parameter("wheel_left", "none");
    // declare_parameter("wheel_right", "none");
    // declare_parameter("wheel_radius", 0.0);
    // declare_parameter("track_width", 0.0);
    // declare_parameter("input_noise", 0.0);
    // declare_parameter("slip_fraction", 0.0);
    declare_parameter("distance_threshold", 0.1);

    // if these are not specified, shutdown the node
    // if (get_parameter("body_id").as_string() == "none") {
    //   RCLCPP_ERROR_STREAM(get_logger(), "No body frame specified.");
    //   rclcpp::shutdown();
    // } else if (get_parameter("wheel_left").as_string() == "none") {
    //   RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint name specified.");
    //   rclcpp::shutdown();
    // } else if (get_parameter("wheel_right").as_string() == "none") {
    //   RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint name specified.");
    //   rclcpp::shutdown();
    // }

    // body_id = get_parameter("body_id").as_string();
    // odom_id = get_parameter("odom_id").as_string();
    // wheel_left = get_parameter("wheel_left").as_string();
    // wheel_right = get_parameter("wheel_right").as_string();
    // const auto radius = get_parameter("wheel_radius").as_double();
    // const auto depth = get_parameter("track_width").as_double();
    distance_threshold = get_parameter("distance_threshold").as_double();

    // turtlelib::DiffDrive tbot{depth, radius};
    // tbot3 = tbot;
    // blue_path_msg.header.frame_id = odom_id;


    // initialize publisher, subscriber, and service
    //odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    point_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("SLAM/points", 10);
    laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/red/base_scan", 10, std::bind(
        &LandmarksNode::lidar_callback, this,
        std::placeholders::_1));
    //initialp_service_ =
    //   create_service<nuturtle_control::srv::InitialPose>(
    //   "/initial_pose",
    //   std::bind(&OdomNode::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  turtlelib::DiffDrive tbot3{0.0, 0.0};
  double distance_threshold{};
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blue_path_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//   rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initialp_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr point_publisher_;

  void on_timer()
  {

  }

  void lidar_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    
    std::vector<double> range_data{};
    double range_datum{};
    for(int i{0}; i < (int) msg.ranges.size(); i++)
    {
        range_datum = msg.ranges.at(i);
        range_data.push_back(range_datum);
    }
    turtlelib::Clusters lidar = turtlelib::clustering(range_data, msg.angle_increment, distance_threshold);
    turtlelib::drop_clusters(lidar);
    
    publish_clusters(lidar);

  }

  void publish_clusters(turtlelib::Clusters cluster)
  {
    visualization_msgs::msg::MarkerArray lidar_data{};
    rclcpp::Time stamp = get_clock()->now();
    for (int i = 0; i < (int) cluster.ranges.size(); ++i) {
        if(cluster.ranges.at(i).cluster !=-1)
        {
            visualization_msgs::msg::Marker point;
            point.header.frame_id = "green/base_footprint";
            point.header.stamp = stamp;
            point.type = visualization_msgs::msg::Marker::SPHERE;
            point.scale.x = .01;
            point.scale.y = .01;
            point.scale.z = 0.01;
            if(cluster.ranges.at(i).cluster == 0)
            {
                point.color.r = 1.0;
            }
            else if(cluster.ranges.at(i).cluster == 1)
            {
                point.color.g = 1.0;
            }
            else if(cluster.ranges.at(i).cluster == 2)
            {
                point.color.b = 1.0;
            }
            else if(cluster.ranges.at(i).cluster == 3)
            {
                point.color.r = 1.0;
                point.color.g = 128.0/255.0;
            }
            else if(cluster.ranges.at(i).cluster == 4)
            {
                point.color.r = 1.0;
                point.color.g = 1.0;
            }
            else if(cluster.ranges.at(i).cluster == 5)
            {
                point.color.r = 1.0;
                point.color.g = 51.0/255.0;
                point.color.b = 1.0;
            }
            point.color.a = 1.0;
            point.id = i;
            point.action = visualization_msgs::msg::Marker::ADD;
            point.pose.position.x = cluster.ranges.at(i).range*cos(cluster.ranges.at(i).angle);
            point.pose.position.y = cluster.ranges.at(i).range*sin(cluster.ranges.at(i).angle);
            lidar_data.markers.push_back(point);
        }
    }

    point_publisher_->publish(lidar_data);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarksNode>());
  rclcpp::shutdown();
  return 0;
}