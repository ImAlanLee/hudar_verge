/*
 * @Author: AlanLee
 * @Date: 2024-08-20 18:20:24
 * @LastEditors: xuxin lisian_magic@163.com
 * @LastEditTime: 2024-08-20 18:29:56
 * @FilePath: /hudar_verge/src/hudar_camera/src/publish_video.cpp
 * @Description:
 */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher() : Node("radar_camera") {

    this->declare_parameter("/radar/video_path", "None");
    std::string video_path =
        this->get_parameter("/radar/video_path").as_string();

    if (video_path == "None") {
      RCLCPP_INFO(this->get_logger(), "Can't Find Video. Exiting!");
      exit(-1);
    }
    // 创建Publisher，发布图像消息到"/image_raw"话题
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/radar/image_raw", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(16), // 以60FPS播放
        std::bind(&ImagePublisher::timer_callback, this));

    // 打开默认摄像头（设备索引0）
    cap_.open(video_path);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
    }

    RCLCPP_INFO(this->get_logger(), "Now publishing from video!");
  }

private:
  void timer_callback() {
    cv::Mat frame;
    cap_ >> frame; // 从摄像头捕获一帧图像
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

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
