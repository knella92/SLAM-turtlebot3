/// \circle.cpp
/// \brief Defines and launche nusim node with appropriate publishers and services
///
/// PARAMETERS:

/// PUBLISHES:

/// SUBSCRIBES:

/// SERVERS:

/// CLIENTS:


#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

class CircleNode : public rclcpp::Node
{
public:
  CircleNode()
  : Node("circle")
  {

    //declare initial parameters
    declare_parameter("frequency", 100);


    const auto frequency = get_parameter("frequency").as_int();
    const int64_t rate = 1000/frequency;
    t = rate;


    // initialize publishers and timer
    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<int64_t, std::milli>(t), std::bind(&CircleNode::timer_callback, this));
   
    control_service_ =
        create_service<nuturtle_control::srv::Control>(
        "/control",
        std::bind(&CircleNode::control, this, std::placeholders::_1, std::placeholders::_2));

    reverse_service_ =
        create_service<std_srvs::srv::Empty>(
        "/reverse",
        std::bind(&CircleNode::reverse, this, std::placeholders::_1, std::placeholders::_2));
  }

private:

    turtlelib::DiffDrive tbot3{0.0,0.0};
    int64_t t{};
    double ang_velocity{0.0};
    double radius{0.0};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;

    void timer_callback()
    {
        geometry_msgs::msg::Twist Vb{};
        Vb.linear.x = ang_velocity*radius;
        Vb.angular.z = ang_velocity;
        vel_publisher_->publish(Vb);
    }

    void control(
        const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
        const std::shared_ptr<nuturtle_control::srv::Control::Response> response){

        ang_velocity = request->velocity;
        radius = request->radius;
        (void)response;
    }

    void reverse(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        ang_velocity *= -1.0;
        (void) request;
        (void) response;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleNode>());
  rclcpp::shutdown();
  return 0;
}
