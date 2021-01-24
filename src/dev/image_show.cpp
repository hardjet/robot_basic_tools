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
  }
}

void ImageShow::show() {
  // 定义窗口
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
  // cv::resizeWindow(window_name_, 640, 480);

  while (is_show_image_) {
    cv_bridge::CvImagePtr cv_ptr;

    // 等待图像就绪
    if (!image_ptr_) {
      usleep(10000);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      // 尝试换换为BGR
      try {
        cv_ptr = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
        // 尝试换换为灰度图
        try {
          cv_ptr = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
          usleep(10000);
          continue;
        }
      }
    }

    // 显示
    cv::imshow(window_name_, cv_ptr->image);
    cv::waitKey(10);
    usleep(50000);
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
