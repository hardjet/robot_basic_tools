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
#include "camera_model/camera_models/Camera.h"

// ---- 相机-单线激光标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 等待新数据
#define STATE_WAIT_DATA 2
// 采集数据
#define STATE_GET_CAM_POSE 3

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
  std::lock_guard<std::mutex> lock(mtx_);
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

bool CamLaserCalib::get_cam_pose() {
  // 检测到的角点空间坐标
  std::vector<cv::Point2f> imagePoints;
  // 检测到的角点图像坐标
  std::vector<cv::Point3f> objectPoints;
  // 相机坐标系到世界坐标系的变换
  Eigen::Matrix4d Twc;
  boost::shared_ptr<const cv_bridge::CvImage> cur_image{nullptr};

  // 将图像锁定
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cur_image.reset(new const cv_bridge::CvImage(image_ptr_->header, image_ptr_->encoding, image_ptr_->image));
  }

  cv::Mat img;
  // 需要转为灰度
  if (cur_image->image.channels() == 3) {
    cv::cvtColor(cur_image->image, img, cv::COLOR_RGB2GRAY);
  } else {
    img = cur_image->image.clone();
  }

  cv::Mat img_show;
  // 需要转为彩色图像
  if (cur_image->image.channels() == 1) {
    cv::cvtColor(cur_image->image, img_show, cv::COLOR_GRAY2RGB);
  } else {
    img_show = cur_image->image.clone();
  }

  if (april_board_ptr_->board->computeObservation(img, img_show, objectPoints, imagePoints)) {
    // 计算外参T_wc cam->world
    cam_ptr_->cam()->estimateExtrinsics(objectPoints, imagePoints, Twc, img_show);

    // std::lock_guard<std::mutex> lock(mtx_);
    // image_mat_.reset(new cv::Mat(img_show.clone()));
    show_image_ptr_.reset(new const cv_bridge::CvImage(cur_image->header, cur_image->encoding, img_show));
    return true;
  } else {
    // std::cout << "no april board detected!" << std::endl;
    return false;
  }
}

void CamLaserCalib::calibration() {
  // 首先获取最新的数据
  update_data();

  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      if (is_new_image_) {
        is_new_image_ = false;
        cur_state_ = STATE_GET_CAM_POSE;
      }
      break;
    case STATE_GET_CAM_POSE:
      // 执行任务
      if (task_ptr_->do_task("get_cam_pose", std::bind(&CamLaserCalib::get_cam_pose, this))) {
        cur_state_ = STATE_IDLE;
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          // std::lock_guard<std::mutex> lock(mtx_);
          im_show_ptr_->update_image(show_image_ptr_);
        }
      }
      break;
  }
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
    ImGui::SameLine();
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &is_show_image_)) {
      if (is_show_image_) {
        im_show_ptr_->enable("calib", false);
      } else {
        im_show_ptr_->disable();
      }
    }

    // 标定逻辑
    calibration();
  }

  if (next_state_ == STATE_IDLE) {
    if (ImGui::Button("start")) {
      next_state_ = STATE_START;
    }
  } else {
    if (ImGui::Button("stop")) {
      next_state_ = STATE_IDLE;
    }
  }

  ImGui::End();

  // 显示图像
  im_show_ptr_->show_image(is_show_image_);
}

}  // namespace calibration
