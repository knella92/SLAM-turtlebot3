/// \landmarks.cpp
/// \brief Defines and launches landmarks node which interprets laser scan data for obstacle recognition
///
/// PARAMETERS:
///     distance_threshold (double): Largest difference in range between laser scan points to be within the same cluster
/// PUBLISHES:
///     (/SLAM/points) (MarkerArray): MarkerArray of recognized obstacle sizes and positions relative to robot
/// SUBSCRIBES:
///     (/red/base_scan) (LaserScan): Laserscan data
/// SERVERS:
///     none
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

    declare_parameter("distance_threshold", 0.05);
    distance_threshold = get_parameter("distance_threshold").as_double();


    // initialize publisher, subscriber, and service
    point_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("SLAM/points", 10);
    laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/red/base_scan", 10, std::bind(
        &LandmarksNode::lidar_callback, this,
        std::placeholders::_1));

  }

private:
  turtlelib::DiffDrive tbot3{0.0, 0.0};
  double distance_threshold{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
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
    std::vector<turtlelib::Vector2D> centroids = turtlelib::centroid_finder(lidar);
    
    turtlelib::ClustersCentroids cluster_points = turtlelib::shift_points(lidar, centroids);
    std::vector<turtlelib::Circle> detected_circles = turtlelib::circle_detection(cluster_points);
    std::vector<bool> is_circle = turtlelib::classification(detected_circles);
    publish_clusters(detected_circles, is_circle);


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
            lidar_data.markers.push_back(obst);
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