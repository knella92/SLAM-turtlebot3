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
    declare_parameter("distance_threshold", 0.05);

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
    // for(int i{0}; i < (int) lidar.ranges.size(); i++)
    // {
    //     RCLCPP_INFO_STREAM(get_logger(), "" << lidar.ranges.at(i).cluster);
    // }
    // if(lidar.n_clusters > 0)
    // {
        std::vector<turtlelib::Vector2D> centroids = turtlelib::centroid_finder(lidar);
        
        turtlelib::ClustersCentroids cluster_points = turtlelib::shift_points(lidar, centroids);
        std::vector<turtlelib::Circle> detected_circles = turtlelib::circle_detection(cluster_points);
        std::vector<bool> is_circle = turtlelib::classification(detected_circles);
        // for(int i{0}; i< (int) is_circle.size(); i++)
        // {
        //     RCLCPP_INFO_STREAM(get_logger(), "" << is_circle);
        // }
        publish_clusters(detected_circles, is_circle);
        // publish_clusters(lidar);
    // }

  }

  void publish_clusters(std::vector<turtlelib::Circle> detected_circles, std::vector<bool> is_circle)
  {
    visualization_msgs::msg::MarkerArray lidar_data{};
    rclcpp::Time stamp = get_clock()->now();
    if((int) detected_circles.size() == 0)
    {
        visualization_msgs::msg::Marker obst{};
        obst.header.frame_id = "red/base_footprint";
        obst.header.stamp = stamp;
        lidar_data.markers.push_back(obst);
    }
    else
    {
        for (int i = 0; i < (int) is_circle.size(); ++i) 
        {
            if(is_circle.at(i) == false)
            {
                visualization_msgs::msg::Marker obst{};
                obst.header.frame_id = "red/base_footprint";
                obst.header.stamp = stamp;
                lidar_data.markers.push_back(obst);
                continue;
            }
            
            // int i = lidar.ranges.at(j).cluster;
            visualization_msgs::msg::Marker obst{};
            obst.header.frame_id = "red/base_footprint";
            obst.header.stamp = stamp;
            obst.type = visualization_msgs::msg::Marker::CYLINDER;
            obst.scale.x = detected_circles.at(i).R;
            obst.scale.y = detected_circles.at(i).R;
            obst.scale.z = .25;
            obst.color.r = .627;
            obst.color.g = 0.125;
            obst.color.b = 0.941;
            obst.color.a = 1.0;
            obst.id = i;
            obst.lifetime = rclcpp::Duration(100ms);
            obst.action = visualization_msgs::msg::Marker::ADD;
            obst.pose.position.x = detected_circles.at(i).a;
            obst.pose.position.y = detected_circles.at(i).b;
            // obst.pose.position.x = lidar.ranges.at(j).range * cos(lidar.ranges.at(j).angle);
            // obst.pose.position.y = lidar.ranges.at(j).range * sin(lidar.ranges.at(j).angle);
            lidar_data.markers.push_back(obst);
            // RCLCPP_INFO_STREAM(get_logger(), "" << detected_circles.at(i).a);
            // k++;
            // RCLCPP_INFO_STREAM(get_logger(), "" << i);
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