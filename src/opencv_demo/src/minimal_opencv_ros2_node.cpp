/*#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include "ImageConverter.cpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
 
class minimal_opencv_ros2_node : public rclcpp::Node {
public:

  minimal_opencv_ros2_node() : Node("opencv_image_publisher") {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/robotcam", 10, std::bind(&ImageConverter::imageCallback, this, std::placeholders::_1));
    angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/current_angle", 10, std::bind(&minimal_opencv_ros2_node::current_angle_callback, this, std::placeholders::_1));
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);

    double resX = 640; //resolution i think
    double FOV = 90; //might be 100? not really sure
  }
 
private:
  sensor_msgs::msg::Image img;
  std_msgs::msg::Float32 angle;
  void minimal_opencv_ros2_node::current_angle_callback(const std_msgs::msg::Float32 ang);
  void minimal_opencv_ros2_node::find_red(const sensor_msgs::msg::Image img);
  void minimal_opencv_ros2_node::angle_calculator();
};
void minimal_opencv_ros2_node::current_angle_callback(const std_msgs::msg::Float32 ang){
  angle = ang;
}
Point minimal_opencv_ros2_node::find_red(const sensor_msgs::msg::Image img){
    cv::Scalar lowerRed1(0, 100, 100);
    cv::Scalar upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(160, 100, 100);
    cv::Scalar upperRed2(179, 255, 255);

    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(img->data[0], lowerRed1, upperRed1, redMask1);
    cv::inRange(img->data[0], lowerRed2, upperRed2, redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);
  
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
    cv::Moments moments = cv::moments(contours[0]);
    cv::Point center(moments.m10 / moments.m00, moments.m01 / moments.m00);

    cv::waitKey(0);
    return center;
  }
void minimal_opencv_ros2_node::angle_calculator(){
  //Calculate angle of camera
    double x = find_red(img).x();
    double PI = 3.14159265; //idk how to get actual pi
    //double angle = FOV/resX*x*PI/180.0+ang;
    auto predictedMSG = std_msgs::msg::Float32();
    predictedMsg.data.resize(1);
    predictedMsg.data[0] = angle->data[0]+FOV/resX*x*PI/180.0;
    angle_publisher_->publish(predictedMsg);
}
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<minimal_opencv_ros2_node>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/