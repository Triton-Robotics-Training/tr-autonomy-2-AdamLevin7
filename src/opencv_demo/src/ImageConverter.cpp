#include "ImageConverter.h"

//much thanks: https://gist.github.com/nightduck/a07c185faad82aeaacbfa87298d035c0
ImageConverter::ImageConverter() : Node("ImageConverter"){
    // this window is just to show openCV changes
    cv::namedWindow("Image window");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    pub_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
    curr_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "current_angle", 10, std::bind(&ImageConverter::angleCallback, this, _1));
    sub_ = image_transport::create_subscription(
            this, "robotcam",std::bind(&ImageConverter::imageCallback,
                                       this, _1), "raw", custom_qos);
}

//callback to subscribe to current angle and put value in curr_angle
void ImageConverter::angleCallback(const std_msgs::msg::Float32::SharedPtr msg){
    curr_angle = msg->data;
}

// image callback which receives and processes image and then publishes angle
void ImageConverter::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    //use cv_bridge to convert ROS image to OpenCV image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    // make sure image is large enough
    float x = 0.0f;
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        // Draw circle
        cv::circle(cv_ptr->image, cv::Point(150, 150), 10, CV_RGB(0,255,0));
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Scalar lower_red1(0, 120, 70); // Lower bound for red
    cv::Scalar upper_red1(10, 255, 255); // Upper bound for red (lower hue)
    cv::Scalar lower_red2(170, 120, 70); // Lower bound for red (upper hue)
    cv::Scalar upper_red2(180, 255, 255); 
    cv::Mat red_hue_image;
    cv::Mat mask1, mask2;
    cv::inRange(hsv_image, lower_red1, upper_red1, mask1); // Mask for lower red range
    cv::inRange(hsv_image, lower_red2, upper_red2, mask2); // Mask for upper red range

    // Combine the two masks
    cv::Mat red_mask;
    cv::bitwise_or(mask1, mask2, red_mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty())
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
      } 
    // Display image with circle
    cv::imshow("Image window", cv_ptr->image);
    cv::waitKey(3);
    
    // Create a message object for publishing with a fixed value
    float PI = 3.14159265f; //idk how to get actual pi
    float resX = 640.0f; //resolution i think
    float FOV = 90.0f; //might be 100? not really sure
    float angle = curr_angle+FOV/resX*(320-x)*PI/180.0;
    auto output_msg = std_msgs::msg::Float32();
    output_msg.data = angle;
    RCLCPP_INFO(this->get_logger(), "its the final value %f", x);
    pub_->publish(output_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}