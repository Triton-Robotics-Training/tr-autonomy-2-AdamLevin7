#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <stdlib.h>
#include <functional>

using namespace std::chrono_literals;
 
class plsWonk2 : public rclcpp::Node {
  
public:
  plsWonk2() : Node("plsWonk2") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);    
    }
  void publishFloat(float value)
      {
        auto msg = std_msgs::msg::Float32();
        msg.data = value;
        publisher_->publish(msg);
      }
private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};


 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<plsWonk2>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  while (rclcpp::ok()) {
    node->publishFloat(3);
    rclcpp::spin_some(node);
}
  
  rclcpp::shutdown();
  return 0;
}
