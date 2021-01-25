#pragma once


#include <thread>
#include <mutex>
#include <sensor_msgs/Image.h>

namespace dev {

class ImageShow {
 public:
  ImageShow() = default;

  ~ImageShow();

  /**
  * @brief 更新当前实现图像
  * @param image
  */
  void update_image(sensor_msgs::ImageConstPtr &image);

  /// 使能图像显示
  void enable(std::string& window_name);

  /// 关闭图像显示
  void disable();

 private:
  /// 显示图像操作，放在单独的线程中
  void show();

 private:
  // 是否显示图片
  bool is_show_image_{false};

  std::mutex mtx_;
  // 显示图片线程
  std::thread thread_;

  // 是否需要将sensor_msgs::Imaga转为cv::mat，减低系统资源占用
  bool is_need_cv_convert_{false};

  // 接收到的图像数据
  sensor_msgs::ImageConstPtr image_ptr_{nullptr};

  // 窗口名称
  std::string window_name_;
};

}  // namespace dev