#pragma once

#include <thread>
#include <mutex>
#include <sensor_msgs/Image.h>

namespace cv_bridge {
class CvImage;
}

namespace dev {

class ImageShow {
 public:
  ImageShow();

  ~ImageShow();

  /**
   * @brief 更新当前图像
   * @param image 图像信息保存在cv::Mat
   */
  void update_image(boost::shared_ptr<cv_bridge::CvImage const>& image);

  /// 使能图像显示
  void enable(const std::string& window_name, bool is_use_opencv = false);

  /// 关闭图像显示
  void disable();

  /// 显示图像
  void show_image(bool& is_show_image);

 private:
  /// 显示图像操作，放在单独的线程中
  void show_in_opencv();

  /// 更新纹理数据，支持灰度和RGB
  void update_texture();

 private:
  // 是否显示图片
  bool is_show_image_{false};

  // 是否使用opencv显示
  bool is_use_opencv_{false};

  // 数据就绪
  bool is_texture_ready_{false};

  // 是否需要更新texture
  bool is_need_update_texture_{false};

  // 资源锁
  std::mutex mtx_;

  // 显示图片opencv线程
  std::thread thread_;

  // 窗口名称
  std::string window_name_;

  // 保存转码后的图像数据
  boost::shared_ptr<cv_bridge::CvImage const> image_cv_ptr_{nullptr};

  // 纹理id
  unsigned int image_texture_{0};
};

}  // namespace dev