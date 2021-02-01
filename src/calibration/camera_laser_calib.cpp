#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "imgui.h"

#include "dev/april_board.hpp"
#include "dev/camera.hpp"
#include "dev/laser.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "calibration/task_back_ground.hpp"
#include "calibration/camera_laser_calib.hpp"
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"

namespace calibration {

CamLaserCalib::CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                             std::shared_ptr<dev::AprilBoard>& april_board_ptr_)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr_) {
  // 图像显示
  im_show_ptr_ = std::make_shared<dev::ImageShow>();
  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
}

void CamLaserCalib::draw_gl(glk::GLSLShader& shader) {}

void CamLaserCalib::update_data() {
  // 图像
  if (!image_ptr_) {
    image_ptr_ = cam_ptr_->data();
    if (image_ptr_) {
      is_new_image_ = true;
    }
  } else {
    auto image = cam_ptr_->data();
    if (image_ptr_->header.stamp.nsec != image->header.stamp.nsec) {
      image_ptr_ = image;
      is_new_image_ = true;
    }
  }
}

bool CamLaserCalib::find_and_draw_apriltags() {
  // 角点的图像坐标
  std::vector<cv::Point2f> corners_2d;
  // 是否检测到角点
  std::vector<bool> is_corner_observed;

  cv::Mat img;
  // 需要转为灰度
  if (image_ptr_->image.channels() == 3) {
    cv::cvtColor(image_ptr_->image, img, cv::COLOR_RGB2GRAY);
  } else {
    img = image_ptr_->image.clone();
  }

  cv::Mat img_show;
  // 需要转为彩色图像
  if (image_ptr_->image.channels() == 1) {
    cv::cvtColor(image_ptr_->image, img_show, cv::COLOR_GRAY2RGB);
  } else {
    img_show = image_ptr_->image.clone();
  }

  if (april_board_ptr_->board->computeObservation(img, img_show, corners_2d, is_corner_observed)) {
    std::lock_guard<std::mutex> lock(mtx_);
    image_mat_.reset(new cv::Mat(img_show.clone()));
    show_image_ptr_.reset(new const cv_bridge::CvImage(image_ptr_->header, image_ptr_->encoding, *image_mat_));
    // std::cout << "find_and_draw_apriltags:" << image_mat_->cols << std::endl;
  } else {
    std::cout << "no april board detected!" << std::endl;
  }
  return true;
}

void CamLaserCalib::draw_ui() {
  if (!is_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Camera and Laser Calibration", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 相机选择
  draw_sensor_selector<dev::Camera>("camera", dev::CAMERA, cam_ptr_);
  // 激光选择
  draw_sensor_selector<dev::Laser>("laser", dev::LASER, laser_ptr_);

  if (cam_ptr_ && laser_ptr_) {
    // 首先获取最新的数据
    update_data();
    if (is_new_image_) {
      is_new_image_ = false;

      // 执行任务
      if (task_ptr_->do_task("find_and_draw_apriltags", std::bind(&CamLaserCalib::find_and_draw_apriltags, this))) {
        // 结束后需要读取结果
        task_ptr_->result<bool>();
        // std::cout << "draw_ui:" << image_mat_->cols << std::endl;
        std::lock_guard<std::mutex> lock(mtx_);
        im_show_ptr_->update_image(show_image_ptr_);
      }
    }
    ImGui::SameLine();
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &is_show_image_)) {
      if (is_show_image_) {
        im_show_ptr_->enable("calib", false);
      } else {
        im_show_ptr_->disable();
      }
    }
  }

  ImGui::End();

  im_show_ptr_->show_image(is_show_image_);
}

}  // namespace calibration
