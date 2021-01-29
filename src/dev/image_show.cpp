#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "imgui.h"
#include "GL/gl3w.h"
#include "dev/image_show.hpp"

namespace dev {

ImageShow::ImageShow() {
  // 申请纹理id
  glGenTextures(1, &image_texture_);
  glBindTexture(GL_TEXTURE_2D, image_texture_);

  // Setup filtering parameters for display
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

ImageShow::~ImageShow() {
  is_show_image_ = false;

  if (thread_.joinable()) {
    thread_.join();
  }
}

void ImageShow::update_image(sensor_msgs::ImageConstPtr& image) {
  // 避免更新重复图像数据
  if (!image_ptr_ || image_ptr_->header.stamp.nsec != image->header.stamp.nsec) {
    std::lock_guard<std::mutex> lock(mtx_);
    image_ptr_ = image;
    is_need_cv_convert_ = true;
  }
}

void ImageShow::update_texture() {
  if (!is_need_update_texture_) return;

  // 灰度
  if (cv_ptr_->image.channels() == 1) {
    glBindTexture(GL_TEXTURE_2D, image_texture_);
    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // 可能有问题
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, cv_ptr_->image.cols, cv_ptr_->image.rows, 0, GL_RED, GL_UNSIGNED_BYTE,
                 cv_ptr_->image.data);
  } else {
    glBindTexture(GL_TEXTURE_2D, image_texture_);
    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cv_ptr_->image.cols, cv_ptr_->image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 cv_ptr_->image.data);
  }
  is_need_update_texture_ = false;
  is_texture_ready_ = true;
}

bool ImageShow::cv_convert() {
  bool res = true;
  if (is_need_cv_convert_) {
    // 尝试转换为BGR
    try {
      // TODO realsense color image原本发送的就是'rgb8'格式，但是imshow显示使用BGR8格式
      // 发现使用cv_bridge内部转换很占cpu，因此放到外面转换
      // 原始msg中的图像就是rgb8格式，这样转换最快
      cv_ptr_ = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
      // 尝试转换为灰度图
      try {
        cv_ptr_ = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception& e) {
        res = false;
      }
    }
    is_need_update_texture_ = true;
    is_need_cv_convert_ = false;
  }

  return res;
}

void ImageShow::show_image(bool &is_show_image) {
  if (!is_show_image || is_use_opencv_) {
    return;
  }

  is_show_image_ = is_show_image;

  std::lock_guard<std::mutex> lock(mtx_);

  // 转换失败 需要等待
  if (!cv_convert()) {
    return;
  }
  update_texture();

  if (!is_texture_ready_) {
    return;
  }

  ImGui::Begin(window_name_.c_str(), &is_show_image, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Image((void*)(intptr_t)image_texture_, ImVec2(float(cv_ptr_->image.cols), float(cv_ptr_->image.rows)));
  ImGui::End();
}

void ImageShow::show() {
  // 定义窗口
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_EXPANDED);
  // cv::resizeWindow(window_name_, 640, 360);

  cv::Mat img_show;
  bool is_need_sleep = false;
  while (is_show_image_) {
    // 失败后需要sleep, 避免长时间占用资源
    if (is_need_sleep) {
      usleep(50000);
      is_need_sleep = false;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      // 等待图像就绪
      if (!image_ptr_) {
        is_need_sleep = true;
        continue;
      }

      // 转换失败 需要等待
      if (!cv_convert()) {
        is_need_sleep = true;
        continue;
      }

      if (cv_ptr_->image.channels() == 3) {
        cv::cvtColor(cv_ptr_->image, img_show, cv::COLOR_RGB2BGR);
        // 显示
        cv::imshow(window_name_, img_show);
      } else {
        cv::imshow(window_name_, cv_ptr_->image);
      }
    }
    cv::waitKey(100);
  }

  cv::destroyWindow(window_name_);
}

void ImageShow::enable(std::string& window_name, bool is_use_opencv) {
  window_name_ = window_name + " Image";
  is_show_image_ = true;
  is_use_opencv_ = is_use_opencv;
  if (is_use_opencv_) {
    thread_ = std::thread(&ImageShow::show, this);
  }
}
void ImageShow::disable() {
  is_show_image_ = false;
  if (is_use_opencv_) {
    thread_.join();
  }
}

}  // namespace dev
