/*
 * @Author: AlanLee
 * @Date: 2024-08-20 16:14:24
 * @LastEditors: xuxin lisian_magic@163.com
 * @LastEditTime: 2024-08-21 00:30:56
 * @FilePath: /hudar_verge/src/hudar_core/src/radar_front.cpp
 * @Description:
 */

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <vector>
using namespace cv;
class RadarFront : public rclcpp::Node {
public:
  RadarFront() : Node("radar_front") {
    cv::namedWindow("radar_front", cv::WINDOW_FREERATIO);
    cv::resizeWindow("radar_front", 1600, 1200);
    cv::setMouseCallback("radar_front", &RadarFront::onMouse, this);
    // 创建订阅者，订阅"/image_raw"话题
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/radar/image_raw", 10,
        std::bind(&RadarFront::image_callback, this, std::placeholders::_1));
  }
  void spin() {
    if (!frame.empty()) {
      cv::imshow("radar_front", frame);
      cv::waitKey(1); // 允许OpenCV处理GUI事件
    }
  }

private:
  // 鼠标回调函数
  static void onMouse(int event, int x, int y, int flags, void *userdata) {

    RadarFront *self = static_cast<RadarFront *>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
      RCLCPP_INFO(self->get_logger(), "Left button clicked at (%d, %d)", x, y);
      self->pts.push_back(cv::Point(x, y));
      self->ptsCount++;
    }
  }
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      // 使用cv_bridge将ROS图像消息转换为OpenCV格式
      cv::Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
      frame = tmp.clone();
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::vector<cv::Point> pts;
  cv::Mat frame;
  std::mutex mutex_;
  int ptsCount = 0;
};

void spin_thread(rclcpp::Node::SharedPtr node) { rclcpp::spin(node); }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<RadarFront> node = std::make_shared<RadarFront>();
  std::thread ros_thread(spin_thread, node);
  while (rclcpp::ok()) {
    node->spin();
  }

  // 将spin放到thread里运行，防止图像获取回调函数阻塞线程导致鼠标回调函数无法运行。
  ros_thread.join();
  rclcpp::shutdown();
  return 0;
}
