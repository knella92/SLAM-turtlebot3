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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"


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
    x0 = get_parameter("x0").as_double();
    y0 = get_parameter("y0").as_double();
    theta0 = get_parameter("theta0").as_double();
    x = x0;
    y = y0;
    theta = theta0;

    // declares obstacle parameters
    declare_parameter("obstacles/x", std::vector<double>({0.0}));
    declare_parameter("obstacles/y", std::vector<double>({0.0}));
    declare_parameter("obstacles/r", 0.0);
    obstacles_x = get_parameter("obstacles/x").as_double_array();
    obstacles_y = get_parameter("obstacles/y").as_double_array();
    obstacles_r = get_parameter("obstacles/r").as_double();
    if (obstacles_x.size() != obstacles_y.size()) {
      rclcpp::shutdown();
    }

    declare_parameter("arena/x_length", 0.0);
    declare_parameter("arena/y_length", 0.0);
    declare_parameter("wall/thickness", 0.0);
    arena_x = get_parameter("arena/x_length").as_double();
    arena_y = get_parameter("arena/y_length").as_double();
    wall_thickness = get_parameter("wall/thickness").as_double();

    x_pos = {(arena_x + wall_thickness)/-2.0, (arena_x + wall_thickness)/2.0, 0.0, 0.0};
    y_pos = {0.0, 0.0, (arena_y+wall_thickness)/-2.0, (arena_y + wall_thickness)/2.0};
    length = {arena_y, arena_y, arena_x, arena_x};
    tf2::Quaternion wx, wy;
    wx.setRPY(0.0, 0.0, 0.0);
    wy.setRPY(0.0, 0.0, turtlelib::PI/2.0);
    orient = {tf2::toMsg(wy), tf2::toMsg(wy), tf2::toMsg(wx), tf2::toMsg(wx)};

    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("motor_cmd_max", 0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto depth = get_parameter("track_width").as_double();

    turtlelib::DiffDrive tbot{depth, radius};
    tbot3 = tbot;

    // saves rate parameter as an int
    rate = get_parameter("rate").as_int();
    const int64_t t = 1000 / rate; // implicit conversion of 1000/rate to int64_t to use as time in ms

    // initialize publishers and timer
    publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    sens_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    obst_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>("red/wheel_cmd", 10, std::bind(&SimNode::cmd_callback, this, std::placeholders::_1));

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
  int rate{};
  double x0, y0, theta0, x, y, theta;
  double motor_cmd_per_rad_sec{};
  double encoder_ticks_per_rad{};
  int motor_cmd_max{};
  std::vector<double> obstacles_x, obstacles_y, x_pos, y_pos, length;
  std::vector<geometry_msgs::msg::Quaternion> orient;
  double obstacles_r;
  double arena_x, arena_y;
  double wall_thickness{};
  double phi_lp{0.0}; double phi_rp{0.0};
  turtlelib::DiffDrive tbot3{0.0,0.0};
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sens_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obst_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr cmd_subscriber_;
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

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    geometry_msgs::msg::TransformStamped t{};
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation = quat;
    tf_broadcaster_->sendTransform(t);

    visualization_msgs::msg::MarkerArray obstacles = add_obstacles();
    visualization_msgs::msg::MarkerArray walls = add_walls();
    obst_publisher_->publish(obstacles);
    wall_publisher_->publish(walls);

    nuturtlebot_msgs::msg::SensorData sens_msg{};
    sens_msg.stamp = get_clock()->now();
    sens_msg.left_encoder = tbot3.phi_l*encoder_ticks_per_rad;
    sens_msg.right_encoder = tbot3.phi_r*encoder_ticks_per_rad;
    sens_publisher_->publish(sens_msg);
  }

  /// \brief Reset service
  /// \param request (empty)
  /// \param response (empty)
  /// void reset(request, response);
  void reset(
    const std::shared_ptr<nusim::srv::Reset::Request>,
    const std::shared_ptr<nusim::srv::Reset::Response>)
  {
    RCLCPP_INFO(get_logger(), "Incoming request to reset");
    count_ = 0;
    x = x0;
    y = y0;
    theta = theta0;
    phi_lp = 0.0;
    phi_rp = 0.0;
    tbot3.phi_l = 0.0;
    tbot3.phi_r = 0.0;
    tbot3.q = {x, y, theta};
    RCLCPP_INFO(get_logger(), "Resetting");
  }

  /// \brief Teleport service
  /// void teleport();
  void teleport(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    const std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    RCLCPP_INFO(get_logger(), "Incoming request to teleport");
    x = request->x;
    y = request->y;
    theta = request->theta;
    RCLCPP_INFO(get_logger(), "Teleporting");
  }

  /// \brief Adds obstacles to MarkerArray for publishing in timer_callback()
  /// \return MarkerArray
  visualization_msgs::msg::MarkerArray add_obstacles()
  {
    visualization_msgs::msg::MarkerArray all_obst;

    const int size_x = obstacles_x.size();
    rclcpp::Time stamp = get_clock()->now();
    for (int i = 0; i < size_x; ++i) {
      visualization_msgs::msg::Marker obst;
      obst.header.frame_id = "nusim/world";
      obst.header.stamp = stamp;
      obst.type = visualization_msgs::msg::Marker::CYLINDER;
      obst.scale.x = obstacles_r;
      obst.scale.y = obstacles_r;
      obst.scale.z = 0.25;
      obst.color.r = 1.0;
      obst.color.a = 1.0;
      obst.id = i;
      obst.action = visualization_msgs::msg::Marker::ADD;
      obst.pose.position.x = obstacles_x.at(i);
      obst.pose.position.y = obstacles_y.at(i);
      all_obst.markers.push_back(obst);
    }
    return all_obst;
  }


  visualization_msgs::msg::MarkerArray add_walls()
  {
    visualization_msgs::msg::MarkerArray all_walls;
    rclcpp::Time stamp = get_clock()->now();
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker walls;
      walls.header.frame_id = "nusim/world";
      walls.header.stamp = stamp;
      walls.type = visualization_msgs::msg::Marker::CUBE;
      walls.scale.x = length.at(i);
      walls.scale.y = wall_thickness;
      walls.scale.z = 0.25;
      walls.color.r = 1.0;
      walls.color.a = 1.0;
      walls.id = i;
      walls.action = visualization_msgs::msg::Marker::ADD;
      walls.pose.position.x = x_pos.at(i);
      walls.pose.position.y = y_pos.at(i);
      walls.pose.orientation = orient.at(i);
      all_walls.markers.push_back(walls);
    }

    return all_walls;
  }


  void cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    double phidot_l{2.84*msg.left_velocity/motor_cmd_max};
    double phidot_r{2.84*msg.right_velocity/motor_cmd_max};
    phi_lp += phidot_l;
    phi_rp += phidot_r;
    tbot3.forward_kin(phi_lp, phi_rp);
    x = tbot3.q.x;
    y = tbot3.q.y;
    theta = tbot3.q.theta;
    tbot3.phi_l = phi_lp;
    tbot3.phi_r = phi_rp;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
