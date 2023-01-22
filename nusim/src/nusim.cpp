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


using namespace std::chrono_literals;

class SimNode : public rclcpp::Node
{
  public:
  
    SimNode()
    : Node("nusim"), count_(0)
    {
      
      //declare/get parameters
      this->declare_parameter("rate", 200);
      this->declare_parameter("x0", 0.0);
      this->declare_parameter("y0", 0.0);
      this->declare_parameter("theta0", 0.0);

      this->declare_parameter("x", 0.0);
      this->declare_parameter("y", 0.0);
      this->declare_parameter("theta", 0.0);

      auto rate = this->get_parameter("rate").as_int();

      int64_t t = 1000/rate; // implicit conversion of 1000/rate to int64_t to use as time in ms
      

      publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      timer_ = this->create_wall_timer(
        std::chrono::duration<int64_t,std::milli>(t), std::bind(&SimNode::timer_callback, this));

      service_ = this->create_service<nusim::srv::Reset>("~/reset", std::bind(&SimNode::reset, this, std::placeholders::_1, std::placeholders::_2));
      tele_service_ = this->create_service<nusim::srv::Teleport>("~/teleport", std::bind(&SimNode::teleport, this, std::placeholders::_1, std::placeholders::_2));

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    }

  private:

    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      count_++;
      message.data = count_;
      //RCLCPP_INFO(this->get_logger(), "Publishing: %lu", message.data);
      publisher_->publish(message);

      auto x = this->get_parameter("x").as_double();
      auto y = this->get_parameter("y").as_double();
      auto theta = this->get_parameter("theta").as_double();

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "red/base_footprint";

      t.transform.translation.x = x;
      t.transform.translation.y = y;

      t.transform.rotation.x = 0;
      t.transform.rotation.y = 0;
      t.transform.rotation.z = theta;
      

      tf_broadcaster_->sendTransform(t);
    }

    void reset(const std::shared_ptr<nusim::srv::Reset::Request> request,
               std::shared_ptr<nusim::srv::Reset::Response> response)
    {
        count_ = 0;
        
        auto x = this->get_parameter("x0").as_double();
        auto y = this->get_parameter("y0").as_double();
        auto theta = this->get_parameter("theta0").as_double();

        this->set_parameter(rclcpp::Parameter("x", x));
        this->set_parameter(rclcpp::Parameter("y", y));
        this->set_parameter(rclcpp::Parameter("theta", theta));

        (void)request;
        (void)response;
        RCLCPP_INFO(this->get_logger(), "Incoming request to reset");
        RCLCPP_INFO(this->get_logger(), "Resetting");
    }

    void teleport(const std::shared_ptr<nusim::srv::Teleport::Request> request,
               std::shared_ptr<nusim::srv::Teleport::Response> response)
    {
      this->set_parameter(rclcpp::Parameter("x", request->x));
      this->set_parameter(rclcpp::Parameter("y", request->y));
      this->set_parameter(rclcpp::Parameter("theta", request->theta));
      (void)response;
    }
    
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<nusim::srv::Reset>::SharedPtr service_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr tele_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}