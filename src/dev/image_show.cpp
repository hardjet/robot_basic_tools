#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "dev/image_show.hpp"

namespace dev {

ImageShow::~ImageShow() {
  is_show_image_ = false;

  if (thread_.joinable()) {
    thread_.join();
  }
}

void ImageShow::update_image(sensor_msgs::ImageConstPtr& image) {
  if (!image_ptr_ || image_ptr_->header.stamp.nsec != image->header.stamp.nsec) {
    std::lock_guard<std::mutex> lock(mtx_);
    image_ptr_ = image;
    is_need_cv_convert_ = true;
  }
}

void ImageShow::show() {
  // 定义窗口
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
  // cv::resizeWindow(window_name_, 640, 360);

  // 保存转换出的图像数据
  cv_bridge::CvImagePtr cv_ptr;
  while (is_show_image_) {
    // 等待图像就绪
    if (!image_ptr_) {
      usleep(50000);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (is_need_cv_convert_) {
        // 尝试转换为BGR
        try {
          cv_ptr = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::BGR8);
          is_need_cv_convert_ = false;
        } catch (cv_bridge::Exception& e) {
          // 尝试转换为灰度图
          try {
            cv_ptr = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::MONO8);
            is_need_cv_convert_ = false;
          } catch (cv_bridge::Exception& e) {
            usleep(50000);
            continue;
          }
        }
      }
    }

    // 显示
    cv::imshow(window_name_, cv_ptr->image);
    cv::waitKey(10);
    usleep(100000);
  }

  cv::destroyWindow(window_name_);
}

void ImageShow::enable(std::string& window_name) {
  window_name_ = window_name;
  is_show_image_ = true;
  thread_ = std::thread(&ImageShow::show, this);
}
void ImageShow::disable() {
  is_show_image_ = false;
  thread_.join();
}

}  // namespace dev
