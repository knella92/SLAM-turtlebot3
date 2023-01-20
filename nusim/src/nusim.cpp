#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"


using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("nusim"), count_(0)
    {
    
    //declare/get rate parameter
    this->declare_parameter("rate", 200);
    auto rate = this->get_parameter("rate").as_int();
    int64_t t = 1000/rate;
    

    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    timer_ = this->create_wall_timer(
    std::chrono::duration<int64_t,std::milli>(t), std::bind(&MinimalPublisher::timer_callback, this));
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
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}