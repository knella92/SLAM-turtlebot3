/// \turtle_control.cpp
/// \brief Defines and launche nusim node with appropriate publishers and services
///
/// PARAMETERS:

/// PUBLISHES:

/// SUBSCRIBES:

/// SERVERS:

/// CLIENTS:


#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "include/turtlelib"



#include "nusim/srv/reset.hpp"
#include "nusim/srv/teleport.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node
{
public:
  SimNode()
  : Node("odometry"), count_(0)
  {

    //declare initial parameters
    declare_parameter("body_id", "0.0");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", 0.0);
    declare_parameter("wheel_right", 0.0);

    // gets aforementioned parameters
    if (get_parameter("body_id" == 0.0)){
      
    }
    const auto body_id = get_parameter("body_id").as_string();
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
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    js_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // initialize subscribers
    js_subscriber_ = create_subscriber<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ControlNode::js_callback, this, std::placeholders::_1));
   
    initialp_service_ =
        create_service<>(
        "~/initial_pose",
        std::bind(&OdomNode::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // initializes braodcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;



  void js_callback()
  {

    wheel_publisher_->publish(message);

  }


  void sens_callback()
  {
    js_publisher_->publish(message);
  }

  void initial_pose()
  {

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
