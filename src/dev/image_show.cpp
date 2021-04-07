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
  b_show_image_ = false;

  if (thread_.joinable()) {
    thread_.join();
  }
}

void ImageShow::update_image(boost::shared_ptr<const cv_bridge::CvImage>& image) {
  // 重复更新检测逻辑放在外面
  std::lock_guard<std::mutex> lock(mtx_);
  image_cv_ptr_ = image;
  b_need_update_texture_ = true;
}

void ImageShow::update_texture() {
  if (!b_need_update_texture_) return;

  // 灰度
  if (image_cv_ptr_->image.channels() == 1) {
    glBindTexture(GL_TEXTURE_2D, image_texture_);
    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // 可能有问题
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, image_cv_ptr_->image.cols, image_cv_ptr_->image.rows, 0, GL_RED,
                 GL_UNSIGNED_BYTE, image_cv_ptr_->image.data);
  } else {
    glBindTexture(GL_TEXTURE_2D, image_texture_);
    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_cv_ptr_->image.cols, image_cv_ptr_->image.rows, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, image_cv_ptr_->image.data);
  }
  b_need_update_texture_ = false;
  is_texture_ready_ = true;
}

void ImageShow::show_image(bool& b_show_image) {
  if (!b_show_image_ || b_use_opencv_) {
    return;
  }

  // 避免直接关闭窗口时两个状态不一致
  b_show_image_ = b_show_image;

  std::lock_guard<std::mutex> lock(mtx_);

  // 更新纹理
  update_texture();

  if (!is_texture_ready_) {
    return;
  }

  ImGui::Begin(window_name_.c_str(), &b_show_image, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Image((void*)(intptr_t)image_texture_,
               ImVec2(float(image_cv_ptr_->image.cols), float(image_cv_ptr_->image.rows)));
  ImGui::End();
}

void ImageShow::show_in_opencv() {
  // 定义窗口
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_EXPANDED);
  // cv::resizeWindow(window_name_, 640, 360);

  cv::Mat img_show;
  bool b_need_sleep = false;
  while (b_show_image_) {
    // 失败后需要sleep, 避免长时间占用资源
    if (b_need_sleep) {
      usleep(50000);
      b_need_sleep = false;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      // 等待图像就绪
      if (!image_cv_ptr_) {
        b_need_sleep = true;
        continue;
      }

      std::cout << "show_in_opencv" << image_cv_ptr_->image.cols << image_cv_ptr_->image.rows << std::endl;

      if (image_cv_ptr_->image.channels() == 3) {
        cv::cvtColor(image_cv_ptr_->image, img_show, cv::COLOR_RGB2BGR);
        // 显示
        cv::imshow(window_name_, img_show);
      } else {
        cv::imshow(window_name_, image_cv_ptr_->image);
      }
    }
    cv::waitKey(100);
  }

  cv::destroyWindow(window_name_);
}

void ImageShow::enable(const std::string& window_name, bool b_use_opencv) {
  window_name_ = window_name + " Image";
  b_show_image_ = true;
  b_use_opencv_ = b_use_opencv;
  if (b_use_opencv_) {
    thread_ = std::thread(&ImageShow::show_in_opencv, this);
  }
}
void ImageShow::disable() {
  b_show_image_ = false;
  if (b_use_opencv_) {
    thread_.join();
  }
}

}  // namespace dev
