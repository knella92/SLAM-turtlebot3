/// \nusim.cpp
/// \brief Defines and launche nusim node with appropriate publishers and services
///
/// PARAMETERS:
///     rate (int): The publishing rate of each timestep in Hz
///     x0 (double): Initial x position of the turtlebot (meters)
///     y0 (double): Initial y position of the turtlebot (meters)
///     theta0 (double): Initial orientation of the turtlebot (radians)
///     x (double): X position as requested by Teleport service (meters)
///     y (double): Y position as requested by Teleport service (meters)
///     theta (double): Orientation as requested by Teleport service (radians)
///     obstacles/x: Vector of x positions of obstacles
///     obstacles/y: Vector of y positions of obstacles
///     obstacles/r: Radius of cylindrical obstacle
/// PUBLISHES:
///     (~/timestep) (UInt64): Timestep of the simulation
///     (~/obstacles) (MarkerArray): Location and size of cylindrical obstacles
/// SUBSCRIBES:
///      none
/// SERVERS:
///     Reset (nusim/srv/Reset.srv): Resets position and orientation of turtlebot to user provided parameters
///     Teleport (nusim/srv/Teleport.srv): Sends the turtlebot to the absolute location and orientation commanded
/// CLIENTS:
///     none

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "nusim/srv/reset.hpp"
#include "nusim/srv/teleport.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;

class SimNode : public rclcpp::Node
{
public:
  SimNode()
  : Node("nusim"), count_(0)
  {

    //declare initial parameters
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);

    // gets aforementioned parameters
    const auto x0 = get_parameter("x0").as_double();
    const auto y0 = get_parameter("y0").as_double();
    const auto theta0 = get_parameter("theta0").as_double();

    // declares x, y, and theta parameters as initial position/orientation parameters
    declare_parameter("x", x0);
    declare_parameter("y", y0);
    declare_parameter("theta", theta0);

    // declares obstacle parameters
    declare_parameter("obstacles/x", std::vector<double>({0.0}));
    declare_parameter("obstacles/y", std::vector<double>({0.0}));
    declare_parameter("obstacles/r", 0.0);

    // test for determining whether obstacles/x and obstacles/y are equal in length
    const std::vector<double> obstacles_x = get_parameter("obstacles/x").as_double_array();
    const std::vector<double> obstacles_y = get_parameter("obstacles/y").as_double_array();
    if (obstacles_x.size() != obstacles_y.size()) {
      rclcpp::shutdown();
    }

    // saves rate parameter as an int
    const auto rate = get_parameter("rate").as_int();

    const int64_t t = 1000 / rate; // implicit conversion of 1000/rate to int64_t to use as time in ms

    // initialize publishers and timer
    publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obst_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<int64_t, std::milli>(t), std::bind(&SimNode::timer_callback, this));
    //intializes services
    reset_service_ =
      create_service<nusim::srv::Reset>(
      "~/reset",
      std::bind(&SimNode::reset, this, std::placeholders::_1, std::placeholders::_2));
    tele_service_ =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&SimNode::teleport, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obst_publisher_;
  rclcpp::Service<nusim::srv::Reset>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr tele_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  /// \brief Initializes timer callback which publishes timestep, turtlebot positions, and obstacle positions at each timestep
  /// void timer_callback();
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    count_++;
    message.data = count_;

    publisher_->publish(message);

    auto x = get_parameter("x").as_double();
    auto y = get_parameter("y").as_double();

    auto theta = get_parameter("theta").as_double();


    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = theta;

    tf_broadcaster_->sendTransform(t);


    visualization_msgs::msg::MarkerArray obstacles = add_obstacles();
    obst_publisher_->publish(obstacles);


  }

  /// \brief Reset service
  /// \param request (empty)
  /// \param response (empty)
  /// void reset(request, response);
  void reset(
    const std::shared_ptr<nusim::srv::Reset::Request> request,
    const std::shared_ptr<nusim::srv::Reset::Response> response)
  {
    count_ = 0;

    auto x = get_parameter("x0").as_double();
    auto y = get_parameter("y0").as_double();
    auto theta = get_parameter("theta0").as_double();

    set_parameter(rclcpp::Parameter("x", x));
    set_parameter(rclcpp::Parameter("y", y));
    set_parameter(rclcpp::Parameter("theta", theta));

    RCLCPP_INFO(get_logger(), "Incoming request to reset");
    RCLCPP_INFO(get_logger(), "Resetting");

    (void)request;
    (void)response;

  }

  /// \brief Teleport service
  /// void teleport();
  void teleport(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    const std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    set_parameter(rclcpp::Parameter("x", request->x));
    set_parameter(rclcpp::Parameter("y", request->y));
    set_parameter(rclcpp::Parameter("theta", request->theta));

    RCLCPP_INFO(get_logger(), "Incoming request to teleport");
    RCLCPP_INFO(get_logger(), "Teleporting");

    (void)request;
    (void)response;
  }

  /// \brief Adds obstacles to MarkerArray for publishing in timer_callback()
  /// \return MarkerArray
  visualization_msgs::msg::MarkerArray add_obstacles()
  {
    visualization_msgs::msg::MarkerArray all_obst;

    const std::vector<double> obstacles_x = get_parameter("obstacles/x").as_double_array();
    const std::vector<double> obstacles_y = get_parameter("obstacles/y").as_double_array();
    const double obstacles_r = get_parameter("obstacles/r").as_double();
    if (obstacles_x.size() != obstacles_y.size()){
      rclcpp::shutdown();
    }
    const int size_x = obstacles_x.size();



    for (int i = 0; i < size_x; ++i) {
      visualization_msgs::msg::Marker obst;
      obst.header.frame_id = "nusim/world";
      obst.header.stamp = get_clock()->now();
      obst.type = visualization_msgs::msg::Marker::CYLINDER;
      obst.scale.x = obstacles_r;
      obst.scale.y = obstacles_r;
      obst.scale.z = 0.25;
      obst.color.r = 1.0;
      obst.color.a = 1.0;
      obst.id = i;
      obst.action = visualization_msgs::msg::Marker::ADD;
      obst.pose.position.x = obstacles_x[i];
      obst.pose.position.y = obstacles_y[i];
      all_obst.markers.push_back(obst);
    }

    return all_obst;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
