#include "../include/Camera.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// 使用ros2 run rqt_image_view rqt_image_view 查看图像
class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher() : Node("radar_camera") {
    // 创建Publisher，发布图像消息到"/image_raw"话题
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/radar/image_raw", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 最高100FPS
        std::bind(&ImagePublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "radar_camera ok!");
  }

private:
  void timer_callback() {

    frame = this->cam.getFrame(); // 从摄像头捕获一帧图像
    if (!frame.empty()) {
      // 将OpenCV的图像转换为ROS的Image消息
      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      sensor_msgs::msg::Image::SharedPtr img_msg =
          cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      // 发布图像消息
      publisher_->publish(*img_msg);
      //   RCLCPP_INFO(this->get_logger(), "Publishing image");
    }
  }
  cv::Mat frame;
  HKCam cam;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
