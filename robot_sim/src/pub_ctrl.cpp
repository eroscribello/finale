#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/iiwa/iiwa_arm_controller/commands", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback, this));
      message.data = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    }
    int i=0;
    int s=0;
    std_msgs::msg::Float64MultiArray message;
    
  private:
    void timer_callback()
    {
      //std_msgs::msg::Float64MultiArray message;
      //message.data = {0,0,0,0,0,0,0};
      if(i%2==0){
      	if(s==0){
      		message.data = {0.7,1.0,0.2,1.8,1.0,2.5,0.1};
      		s=1;
      	}else{
      		message.data = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
      		s=0;
      	}
      	
      	RCLCPP_INFO(this->get_logger(), "s = %d",s);
      
      }
      
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing... %f %f %f %f %f %f %f",message.data[0],message.data[1],message.data[2],message.data[3],message.data[4],message.data[5],message.data[6]);
      publisher_->publish(message);
      i=i+1;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
