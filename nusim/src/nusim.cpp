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
///     (~/path): path taken by red robot
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
#include <random>

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
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

class SimNode : public rclcpp::Node
{
public:
  SimNode()
  : Node("nusim"), count_(0)
  {

    // initial pose parameters
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    x0 = get_parameter("x0").as_double();
    y0 = get_parameter("y0").as_double();
    theta0 = get_parameter("theta0").as_double();
    x = x0;
    y = y0;
    theta = theta0;

    // obstacle parameters
    declare_parameter("obstacles/x", std::vector<double>({0.0}));
    declare_parameter("obstacles/y", std::vector<double>({0.0}));
    declare_parameter("obstacles/r", 0.0);
    obstacles_x = get_parameter("obstacles/x").as_double_array();
    obstacles_y = get_parameter("obstacles/y").as_double_array();
    obstacles_r = get_parameter("obstacles/r").as_double();
    if (obstacles_x.size() != obstacles_y.size()) {
      rclcpp::shutdown();
    }

    // wall parameters
    declare_parameter("arena/x_length", 0.0);
    declare_parameter("arena/y_length", 0.0);
    declare_parameter("wall/thickness", 0.0);
    arena_x = get_parameter("arena/x_length").as_double();
    arena_y = get_parameter("arena/y_length").as_double();
    wall_thickness = get_parameter("wall/thickness").as_double();
    x_pos = {(arena_x + wall_thickness) / -2.0, (arena_x + wall_thickness) / 2.0, 0.0, 0.0};
    y_pos = {0.0, 0.0, (arena_y + wall_thickness) / -2.0, (arena_y + wall_thickness) / 2.0};
    length = {arena_y, arena_y, arena_x, arena_x};
    tf2::Quaternion wx, wy;
    wx.setRPY(0.0, 0.0, 0.0);
    wy.setRPY(0.0, 0.0, turtlelib::PI / 2.0);
    orient = {tf2::toMsg(wy), tf2::toMsg(wy), tf2::toMsg(wx), tf2::toMsg(wx)};


    // turtlebot parameters
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("motor_cmd_max", 0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("collision_radius", 0.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto track_width = get_parameter("track_width").as_double();
    collision_radius = get_parameter("collision_radius").as_double();
    turtlelib::Config q{x0, y0, theta0};
    turtlelib::DiffDrive tbot{track_width, radius, q};
    tbot3 = tbot;

    // noise parameters
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    const auto input_noise = get_parameter("input_noise").as_double();
    std::normal_distribution<double> w(0.0, input_noise);
    w_i = w;
    slip_fraction = get_parameter("slip_fraction").as_double();
    std::uniform_real_distribution<double> distr(-slip_fraction, slip_fraction);
    n_i = distr;

    // sensor parameters
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 0.0);
    const auto basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    std::normal_distribution<double> sv(0.0, basic_sensor_variance);
    sens_var = sv;
    max_range = get_parameter("max_range").as_double();

    // Laser scan parameters
    declare_parameter("angle_increment", 0.01745329238474369);
    declare_parameter("range_min", 0.11999999731779099);
    declare_parameter("range_max", 2.0);
    //declare_parameter("sample_N", )
    //declare_parameter("resolution", )
    angle_increment = get_parameter("angle_increment").as_double();
    range_min = get_parameter("range_min").as_double();
    range_max = get_parameter("range_max").as_double();

    //draw only parameter
    declare_parameter("draw_only", false);
    const auto draw_only = get_parameter("draw_only").as_bool();

    // saves rate parameter as an int
    rate = get_parameter("rate").as_int();
    const int64_t t = 1000 / rate; // implicit conversion of 1000 / rate to int64_t to use as time in ms
    sens_rate = rate / 5;

    path_msg.header.frame_id = "nusim/world";
    // initialize publishers and timer
    if (draw_only) {
      timer_ = create_wall_timer(
        std::chrono::duration<int64_t, std::milli>(t),
        std::bind(&SimNode::draw_only_callback, this));
      obst_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
      wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    } else {
      publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      sens_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
      obst_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
      wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
      path_publisher_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
      fake_sensor_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/fake_sensor", 10);
      laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("red/base_scan", 10);
      cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd", 10, std::bind(
          &SimNode::cmd_callback, this,
          std::placeholders::
          _1));

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
      redbot_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

private:
  size_t count_;
  int rate{}; int sens_rate{};
  int i{};
  double dt{};
  double range_min{}; double range_max{}; double angle_increment{}; double sample_N{};
  double resolution{};
  int prev_time = get_clock()->now().nanoseconds();
  int current_time{};
  double x0{}; double y0{}; double theta0{}; double x{}; double y{}; double theta{};
  double v_l{}; double v_r{};
  double phidot_l{}; double phidot_r{};
  double motor_cmd_per_rad_sec{}; double encoder_ticks_per_rad{}; int motor_cmd_max{};
  int index{};
  nav_msgs::msg::Path path_msg;

  std::vector<double> obstacles_x{};
  std::vector<double> obstacles_y{};
  std::vector<double> x_pos{}; std::vector<double> y_pos{}; std::vector<double> length{};
  std::vector<geometry_msgs::msg::Quaternion> orient{};
  std::normal_distribution<double> w_i{};
  std::normal_distribution<double> sens_var{};
  std::uniform_real_distribution<double> n_i{};
  double obstacles_r{};
  double arena_x{}; double arena_y{};
  double wall_thickness{};
  double phi_lp{0.0}; double phi_rp{0.0};
  double input_noise{}; double slip_fraction;
  double max_range{}; double collision_radius{};
  turtlelib::DiffDrive tbot3{0.0, 0.0};

  // initialize publishers, subscribers, timer, services
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sens_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obst_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr cmd_subscriber_;
  rclcpp::Service<nusim::srv::Reset>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr tele_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> redbot_broadcaster_;


  /// \brief Initializes timer callback which publishes timestep, turtlebot positions, and obstacle positions at each timestep
  /// void timer_callback();
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    count_++;
    message.data = count_;
    publisher_->publish(message);

    latching();

    // create and send transform between world and red robot
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
    redbot_broadcaster_->sendTransform(t);

    publish_path(quat);

    // publish sensor message for odometry
    auto sens_msg = nuturtlebot_msgs::msg::SensorData();
    sens_msg.stamp = get_clock()->now();
    sens_msg.left_encoder = tbot3.phi_l * encoder_ticks_per_rad;
    sens_msg.right_encoder = tbot3.phi_r * encoder_ticks_per_rad;
    sens_publisher_->publish(sens_msg);

    // publishing of environment
    visualization_msgs::msg::MarkerArray red_obst = add_obstacles(0, "nusim/world");
    obst_publisher_->publish(red_obst);
    visualization_msgs::msg::MarkerArray red_walls = add_walls();
    wall_publisher_->publish(red_walls);
    if (count_ % sens_rate == 0) {
      visualization_msgs::msg::MarkerArray sens_obst = add_obstacles(1, "red/base_footprint");
      fake_sensor_publisher_->publish(sens_obst);

      lidar();
    }

    if (index == 100) {
      path_msg.header.stamp = get_clock()->now();
      path_publisher_->publish(path_msg);
      index = 0;
    } else {
      path_msg.poses.push_back(publish_path(quat));
      index++;
    }

  }

  void draw_only_callback()
  {
    // publishing of environment
    visualization_msgs::msg::MarkerArray red_obst = add_obstacles(0, "red");
    obst_publisher_->publish(red_obst);
    visualization_msgs::msg::MarkerArray red_walls = add_walls();
    wall_publisher_->publish(red_walls);
  }


  void cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    const auto u_l = msg.left_velocity / motor_cmd_per_rad_sec;
    const auto u_r = msg.right_velocity / motor_cmd_per_rad_sec;

    if (turtlelib::almost_equal(u_l, 0.0) && turtlelib::almost_equal(u_r, 0.0)) {
      v_l = u_l;
      v_r = u_r;
    } else {
      v_l = u_l + w_i(turtlelib::get_random());
      v_r = u_r + w_i(turtlelib::get_random());
    }

    i = 0;
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


  /// \brief Adds obstacles to MarkerArray
  /// \return MarkerArray of obstacles
  visualization_msgs::msg::MarkerArray add_obstacles(int sensor_ind, std::string frame_id)
  {
    visualization_msgs::msg::MarkerArray all_obst{};

    const int size_x = obstacles_x.size();
    rclcpp::Time stamp = get_clock()->now();
    for (int i = 0; i < size_x; ++i) {

      visualization_msgs::msg::Marker obst;
      obst.header.frame_id = frame_id;
      obst.header.stamp = stamp;
      obst.type = visualization_msgs::msg::Marker::CYLINDER;
      obst.scale.x = obstacles_r;
      obst.scale.y = obstacles_r;
      obst.scale.z = 0.25;
      obst.color.r = 1.0;
      obst.color.a = 1.0;
      obst.id = i;

      if (sensor_ind == 1) {

        turtlelib::Transform2D T_wr{{tbot3.q.x, tbot3.q.y}, tbot3.q.theta};
        turtlelib::Transform2D T_wo{{obstacles_x.at(i), obstacles_y.at(i)}, 0.0};
        turtlelib::Transform2D T_ro = T_wr.inv() * T_wo;
        obst.pose.position.x = T_ro.translation().x + sens_var(turtlelib::get_random());
        obst.pose.position.y = T_ro.translation().y + sens_var(turtlelib::get_random());
        // obst.lifetime = rclcpp::Duration(400ms);

        if (sqrt(
            std::pow(
              obst.pose.position.x,
              2.0) + std::pow(obst.pose.position.y, 2.0)) < max_range)
        {
          obst.color.g = 0.917;
          obst.action = visualization_msgs::msg::Marker::ADD;
          // RCLCPP_INFO_STREAM(get_logger(), "obst.pose.position.x: " << obst.pose.position.x);
        } else {
          
          obst.action = visualization_msgs::msg::Marker::DELETE;
          // RCLCPP_INFO_STREAM(get_logger(), "obst.pose.position.x: " << obst.pose.position.x);
        }
      } else {
        obst.action = visualization_msgs::msg::Marker::ADD;
        obst.pose.position.x = obstacles_x.at(i);
        obst.pose.position.y = obstacles_y.at(i);
      }
      all_obst.markers.push_back(obst);
    }

    return all_obst;
  }

  /// \brief - Add walls to MarkerArray for publishing in timer_callback() 
  /// \return MarkerArray of walls
  visualization_msgs::msg::MarkerArray add_walls()
  {
    visualization_msgs::msg::MarkerArray all_walls{};
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

  /// \brief - function that calculates next robot configuration based on last recorded wheel command 
  void latching()
  {
    if (i < 15) {
      tbot3.forward_kin(v_l / rate, v_r / rate);
      const auto dphi_l = (v_l * (1 + n_i(turtlelib::get_random())) / rate);
      const auto dphi_r = (v_r * (1 + n_i(turtlelib::get_random())) / rate);
      tbot3.update_wheel_pose(dphi_l, dphi_r);
      tbot3.q =
        collision_detection(tbot3.q, collision_radius, obstacles_x, obstacles_y, obstacles_r);
      // update position of robot
      x = tbot3.q.x;
      y = tbot3.q.y;
      theta = tbot3.q.theta;
      i++;
    }
  }

  /// \brief - Publishes path of red robot
  /// \param quat - quaternion representing orientation of robot
  /// \return PoseStamped of robot for path publishing
  geometry_msgs::msg::PoseStamped publish_path(geometry_msgs::msg::Quaternion quat)
  {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = get_clock()->now();
    pose_msg.header.frame_id = "red/base_footprint";
    pose_msg.pose.position.x = tbot3.q.x;
    pose_msg.pose.position.y = tbot3.q.y;
    pose_msg.pose.orientation = quat;

    return pose_msg;
  }

  void lidar()
  {
    sensor_msgs::msg::LaserScan msg{};
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "red/base_scan";
    msg.angle_min = 0.0;
    msg.angle_max = 6.2657318115234375;
    msg.angle_increment = angle_increment;
    //msg.time_increment = 0.0005574136157520115;
    msg.scan_time = 0.20066890120506287;
    msg.range_min = range_min;
    msg.range_max = range_max;

    std::vector<float> ranges{};
    for (int i = 0; i < 360; i++) {
      auto range = turtlelib::range_obstacles(
        tbot3.q, range_max, range_min, obstacles_x, obstacles_y,
        obstacles_r, i * angle_increment);
      if (range == 0.0) {
        range = turtlelib::range_walls(tbot3.q, range_max, arena_x, arena_y, i * angle_increment);
        if(range < range_min)
        {
          range = 0.0;
        }
      }
      
      range += sens_var(turtlelib::get_random());
      ranges.push_back(range);
    }
    msg.ranges = ranges;

    laser_scan_publisher_->publish(msg);
  }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
