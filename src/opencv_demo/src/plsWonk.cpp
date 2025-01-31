#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <stdlib.h>
#include <functional>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;
 
class plsWonk : public rclcpp::Node {
  
public:
  plsWonk() : Node("plsWonk") {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/robotcam", 10,
        std::bind(&plsWonk::imageCallback, this, std::placeholders::_1));
    angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/current_angle", 10, std::bind(&plsWonk::angleCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&plsWonk::timer_callback, this));
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);    
    }
  
//    image_transport::ImageTransport it;
  //cv::namedWindow(std::string "OPENCV_WINDOW", cv::WINDOW_AUTOSIZE);

  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  //pub_ = image_transport::create_publisher(this, "out_image_base_topic", custom_qos);
  
  
  /*~plsWonk()
    {
        cv::destroyWindow(std::string "OPENCV_WINDOW");
    } */
 
private:

//    image_transport::ImageTransport it;
  //image_transport::Subscriber sub_;

  //image_transport::Publisher pub_;
  //void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  std_msgs::msg::Float32::SharedPtr angle;
  sensor_msgs::msg::Image::ConstSharedPtr img;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void angleCallback(const std_msgs::msg::Float32::SharedPtr ang){
    RCLCPP_INFO(this->get_logger(), "Received value: %f", ang->data);
    angle = ang;
}
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    img = msg;
  }
  void timer_callback(){
    try
  {
    if(img != nullptr){
      RCLCPP_INFO(this->get_logger(), "Received image with height: %d, width: %d", img->height, img->width);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    

    // Find red pixels
    cv::Mat lower_red_mask;
    cv::Mat upper_red_mask;
    cv::inRange(image, cv::Scalar(0, 0, 100), cv::Scalar(100, 100, 255), lower_red_mask); 
    cv::inRange(image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_mask);

    // Find contours of red regions
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(upper_red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Point center;
    cv::Moments m = cv::moments(contours[0]);
    center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
    double x = center.x;

    // Calculate center of the largest red contour
    /*if (!contours.empty())
    {
      int max_area = 0;
      cv::Point center;
      for (const auto &contour : contours)
      {
        int area = cv::contourArea(contour);
        if (area > max_area)
        {
          max_area = area;
          cv::Moments m = cv::moments(contour);
          center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
        }
      }
      x = center.x;
      RCLCPP_INFO(this->get_logger(), "Center of red object: (%d, %d)", center.x, center.y);
    } */
  double PI = 3.14159265; //idk how to get actual pi
  double resX = 640; //resolution i think
  double FOV = 90; //might be 100? not really sure
  //double angle = FOV/resX*x*PI/180.0+ang;
  float value = angle->data+FOV/resX*x*PI/180.0;
  cv::waitKey(0);
  auto msg = std_msgs::msg::Float32();
  msg.data = value;
  angle_publisher_->publish(msg);
  }
    }
    
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  }

  //void plsWonk::find_red(const sensor_msgs::msg::Image img);

  //rclcpp::Subscription<sensor_msgs::msg::Image::ConstSharedPtr &msg>:: img_subscription_;

};


/*void plsWonk::find_red(const sensor_msgs::msg::Image img){
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
    double x = center.x();
    double PI = 3.14159265; //idk how to get actual pi
    double resX = 640; //resolution i think
    double FOV = 90; //might be 100? not really sure
    //double angle = FOV/resX*x*PI/180.0+ang;
    float value = ang->data+FOV/resX*x*PI/180.0;
    cv::waitKey(0);
    auto msg = std_msgs::msg::Float32();
    msg.data = value;
    angle_publisher_->publish(msg);
  } */
  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // create a ros2 node
    auto node = std::make_shared<plsWonk>();
  
    // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  
    // process ros2 callbacks until receiving a SIGINT (ctrl-c)
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
}
    rclcpp::shutdown();
  return 0;
}