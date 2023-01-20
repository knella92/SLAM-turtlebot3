#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "nusim/srv/reset.hpp"


using namespace std::chrono_literals;

class SimNode : public rclcpp::Node
{
  public:
    SimNode()
    : Node("nusim"), count_(0)
    {
    
    //declare/get rate parameter
    this->declare_parameter("rate", 200);
    auto rate = this->get_parameter("rate").as_int();
    int64_t t = 1000/rate;
    

    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    timer_ = this->create_wall_timer(
    std::chrono::duration<int64_t,std::milli>(t), std::bind(&SimNode::timer_callback, this));

    service_ = this->create_service<nusim::srv::Reset>("reset", std::bind(&SimNode::reset, this, std::placeholders::_1, std::placeholders::_2));

    }

  private:

    void timer_callback()
    {
    auto message = std_msgs::msg::UInt64();
    count_++;
    message.data = count_;
    RCLCPP_INFO(this->get_logger(), "Publishing: %lu", message.data);
    publisher_->publish(message);
    }

    void reset(const std::shared_ptr<nusim::srv::Reset::Request> request,
               std::shared_ptr<nusim::srv::Reset::Response> response)
    {
        count_ = 0;
        (void)request;
        (void)response;
        RCLCPP_INFO(this->get_logger(), "Incoming request to reset");
        RCLCPP_INFO(this->get_logger(), "Resetting");
    }
    
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<nusim::srv::Reset>::SharedPtr service_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}