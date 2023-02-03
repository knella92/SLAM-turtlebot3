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
#include "include/turtlelib"



#include "std_msgs/msg/u_int64.hpp"
#include "nusim/srv/reset.hpp"
#include "nusim/srv/teleport.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  SimNode()
  : Node("turtle_control"), count_(0)
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

    // saves rate parameter as an int
    const auto rate = get_parameter("rate").as_int();
    const int64_t t = 1000 / rate; // implicit conversion of 1000/rate to int64_t to use as time in ms

    // initialize publishers and timer
    wheel_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);
    js_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // initialize subscribers
    vel_subscriber_ = create_subscriber<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&ControlNode::cmd_callback, this, std::placeholders::_1));
    sens_subscriber_ = create_subscriber<nuturtlebot_msgs::msg::SensorData>("/sensor_data", 10, std::bind(&ControlNode::sens_callback, this, std::placeholders::_1);)

  }

private:
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_publisher_;
  rclcpp::Subscriber<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;
  rclcpp::Subscriber<nuturtlebot_msgs::msg::SensorData>::SharedPtr sens_subscriber;


  void cmd_callback()
  {

    wheel_publisher_->publish(message);

  }


  void sens_callback()
  {
    js_publisher_->publish(message);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
