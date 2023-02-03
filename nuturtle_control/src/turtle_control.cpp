/// \turtle_control.cpp
/// \brief Defines and launche nusim node with appropriate publishers and services
///
/// PARAMETERS:

/// PUBLISHES:

/// SUBSCRIBES:

/// SERVERS:

/// CLIENTS:


#include <chrono>
#include <functional>
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


using namespace std::chrono_literals;

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

    //gets aforementioned parameters
    const auto radius = get_parameter("wheel_radius").as_double();
    const auto depth = get_parameter("track_width").as_double();
    const auto mcmd_rads = get_parameter("motor_cmd_per_rad_sec").as_double();

    turtlelib::DiffDrive tbot{depth, radius};
    tbot3 = tbot;
    motor_cmd_per_rad_sec = mcmd_rads;


    // initialize publishers and timer
    wheel_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);
    js_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // initialize subscribers
    vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&ControlNode::cmd_callback, this, std::placeholders::_1));
    // sens_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>("/sensor_data", 10, std::bind(&ControlNode::sens_callback, this));

  }

private:

  turtlelib::DiffDrive tbot3{0.0,0.0};
  double motor_cmd_per_rad_sec{0.0};
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sens_subscriber_;


  void cmd_callback(const geometry_msgs::msg::Twist & msg)
  {
    turtlelib::Twist2D Vb{};
    Vb.v.x = msg.linear.x;
    Vb.v.y = msg.linear.y;
    Vb.w = msg.angular.z;
    
    turtlelib::Wheel_Vel phidot = tbot3.inverse_kin(Vb);
    int32_t left_vel = phidot.l*motor_cmd_per_rad_sec;
    int32_t right_vel = phidot.l*motor_cmd_per_rad_sec;
    auto message = nuturtlebot_msgs::msg::WheelCommands();
    message.left_velocity = left_vel;
    message.right_velocity = right_vel;

    wheel_publisher_->publish(message);

  }


  // void sens_callback()
  // {
  //   //js_publisher_->publish(message);
  // }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
