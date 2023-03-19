/// \turtle_control.cpp
/// \brief Defines and launches the control interface for the turtlebot (both simulation and real)
///
/// PARAMETERS:
///      wheel_radius (double): Radius of the turtlebot's wheels
///      track_width (double): Distance between the turtlebot's wheels
///      motor_cmd_per_rad_sec (double): Conversion factor from radians/second to "motor commands"
///      encoder_ticks_per_rad (double): Conversion factor from radians to encoder ticks
///      motor_cmd_max (double): Maximum motor command integer
/// PUBLISHES:
///     (/wheel_cmd) (WheelCommands): Integer value between -265 and 265 denoting command to motors
///     (/joint_states) (JointState): Joint position and velocity messages
/// SUBSCRIBES:
///     (/cmd_vel) (Twist): Vector with x, y, and rotational velocities in radians/second to be converted to wheel commands
///     (/sensor_data) (SensorData): Encoder position messages in ticks
/// SERVERS:
///     none
/// CLIENTS:
///     none


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("turtle_control")
  {

    //declare initial parameters
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("motor_cmd_max", 0);

    //gets aforementioned parameters
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto track_width = get_parameter("track_width").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();

    const turtlelib::DiffDrive tbot{track_width, radius};
    tbot3 = tbot;

    // initialize publishers and timer
    cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);
    js_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);

    // initialize subscribers
    vel_subscriber_ =
      create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ControlNode::vel_callback, this, std::placeholders::_1));
    sens_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "/sensor_data", 40, std::bind(
        &ControlNode::sens_callback, this,
        std::placeholders::_1));

  }

private:
  turtlelib::DiffDrive tbot3{0.0, 0.0};
  double motor_cmd_per_rad_sec{0.0};
  double encoder_ticks_per_rad{0.0};
  int motor_cmd_max{0};
  rclcpp::Time prev_time{get_clock()->now()};
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sens_subscriber_;


  void vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    // convert to Twist2D for inverse kinematics function
    turtlelib::Twist2D Vb{};
    Vb.v.x = msg.linear.x;
    Vb.v.y = msg.linear.y;
    Vb.w = msg.angular.z;


    turtlelib::Wheel_Vel phidot = tbot3.inverse_kin(Vb);
    // convert rad/s to motor command (wheel command)
    int left_cmd = phidot.l * motor_cmd_per_rad_sec;
    int right_cmd = phidot.r * motor_cmd_per_rad_sec;
    // "normalizes" wheel command
    auto message = nuturtlebot_msgs::msg::WheelCommands();
    if (left_cmd > motor_cmd_max) {
      left_cmd = motor_cmd_max;
    } else if (left_cmd < -motor_cmd_max) {
      left_cmd = -motor_cmd_max;
    }
    if (right_cmd > motor_cmd_max) {
      right_cmd = motor_cmd_max;
    } else if (right_cmd < -motor_cmd_max) {
      right_cmd = -motor_cmd_max;
    }

    message.left_velocity = left_cmd;
    message.right_velocity = right_cmd;

    cmd_publisher_->publish(message);

  }

  void sens_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensor_msgs::msg::JointState message{};
    auto dt = message.header.stamp.nanosec - prev_time.nanoseconds();
    turtlelib::Wheel_Vel phidot{};
    phidot.l = (msg.left_encoder / encoder_ticks_per_rad - tbot3.phi_l) / (1e-9 * dt);
    tbot3.phi_l = msg.left_encoder / encoder_ticks_per_rad;
    phidot.r = (msg.right_encoder / encoder_ticks_per_rad - tbot3.phi_r) / (1e-9 * dt);
    tbot3.phi_r = msg.right_encoder / encoder_ticks_per_rad;

    message.name = {"wheel_left_joint", "wheel_right_joint"};
    message.position = {tbot3.phi_l, tbot3.phi_r};
    message.velocity = {phidot.l, phidot.r};
    message.header.stamp = get_clock()->now();
    js_publisher_->publish(message);
    prev_time = message.header.stamp;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
