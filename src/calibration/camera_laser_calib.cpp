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
#include "algorithm/line.hpp"
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/camera_models/Camera.h"

// ---- 相机-单线激光标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取相机位姿和落在标定板上的激光点
#define STATE_GET_POSE_AND_PTS 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3

#define DEG2RAD(x) (x * M_PI / 180.0)
#define RAD2DEG(x) (x * 180.0 / M_PI)

namespace calibration {

CamLaserCalib::CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                             std::shared_ptr<dev::AprilBoard>& april_board_ptr_)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr_) {
  // 图像显示
  im_show_dev_ptr_ = std::make_shared<dev::ImageShow>();
  // 激光以图像显示
  laser_show_dev_ptr_ = std::make_shared<dev::ImageShow>();
  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
  // 当前标定数据
  calib_data_ = std::make_shared<CalibData>();
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

  // 激光
  if (!laser_data_ptr_) {
    laser_data_ptr_ = laser_ptr_->data();
    if (laser_data_ptr_) {
      is_new_laser_ = true;
    }
  } else {
    auto laser = laser_ptr_->data();
    if (laser_data_ptr_->header.stamp.nsec != laser->header.stamp.nsec) {
      laser_data_ptr_ = laser;
      is_new_laser_ = true;
    }
  }
}

bool CamLaserCalib::get_pose_and_points() {
  // 检测到的角点空间坐标
  std::vector<cv::Point2f> imagePoints;
  // 检测到的角点图像坐标
  std::vector<cv::Point3f> objectPoints;
  // 相机坐标系到世界坐标系的变换
  Eigen::Matrix4d Twc;

  // 当前处理的图像以及激光数据
  boost::shared_ptr<const cv_bridge::CvImage> cur_image{nullptr};
  boost::shared_ptr<const sensor_msgs::LaserScan> cur_laser_data{nullptr};

  auto start_time = ros::WallTime::now();
  // 将图像和激光锁定
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cur_image.reset(new const cv_bridge::CvImage(image_ptr_->header, image_ptr_->encoding, image_ptr_->image));
    cur_laser_data = laser_data_ptr_;
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
    calib_data_->timestamp = cur_image->header.stamp.toSec();
    calib_data_->t_wc = Twc.block(0, 3, 3, 1);
    Eigen::Matrix3d R_wc = Twc.block(0, 0, 3, 3);
    Eigen::Quaterniond q_wc(R_wc);
    calib_data_->q_wc = q_wc;

    // std::lock_guard<std::mutex> lock(mtx_);
    show_cam_img_ptr_.reset(new const cv_bridge::CvImage(cur_image->header, cur_image->encoding, img_show));
    calib_data_->cam_img_ptr_ = show_cam_img_ptr_;

    // 检测激光中的直线
    cv::Mat laser_show;
    algorithm::Line line(*cur_laser_data, 60.0, 2.0);
    if (line.find_line(calib_data_->line_pts, laser_show)) {
      auto end_time = ros::WallTime::now();
      double time_used = (end_time - start_time).toSec() * 1000;

      // 显示用时
      cv::putText(laser_show, "time used (ms):  " + std::to_string(time_used), cv::Point2f(10, 20),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));

      show_laser_img_ptr_.reset(new const cv_bridge::CvImage(cur_laser_data->header, cur_image->encoding, laser_show));
      calib_data_->laser_img_ptr_ = show_laser_img_ptr_;
      return true;
    } else {
      return false;
    }

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
      if (is_new_image_ && is_new_laser_) {
        is_new_image_ = false;
        is_new_laser_ = false;
        cur_state_ = STATE_GET_POSE_AND_PTS;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_POSE_AND_PTS:
      // 执行任务
      if (task_ptr_->do_task("get_pose_and_points", std::bind(&CamLaserCalib::get_pose_and_points, this))) {
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          // std::lock_guard<std::mutex> lock(mtx_);
          im_show_dev_ptr_->update_image(show_cam_img_ptr_);
          laser_show_dev_ptr_->update_image(show_laser_img_ptr_);
          cur_state_ = STATE_CHECK_STEADY;
        } else {
          cur_state_ = STATE_IDLE;
        }
      }
      break;
    case STATE_CHECK_STEADY:
      if (calib_data_vec_.empty()) {
        calib_data_vec_.push_back(*calib_data_);
      } else {
        // 检查相机位姿是否一致
        double dist = (calib_data_vec_.at(0).t_wc - calib_data_->t_wc).norm();
        // 四元数的转角是原本的1/2
        double theta = 2 * std::acos((calib_data_vec_.at(0).q_wc.inverse() * calib_data_->q_wc).w());
        std::cout << "dist:" << dist << ", theta:" << RAD2DEG(theta) << std::endl;
        // 抖动小于1cm与2°
        if (dist < 0.01 && theta < DEG2RAD(0.5)) {
          calib_data_vec_.push_back(*calib_data_);
        } else {
          // 抖动大则重新开始检测
          calib_data_vec_.clear();
          calib_data_vec_.push_back(*calib_data_);
          std::cout << "moved!!!" << std::endl;
        }
        cur_state_ = STATE_IDLE;
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
        im_show_dev_ptr_->enable("calib cam", false);
        laser_show_dev_ptr_->enable("calib laser", false);
      } else {
        im_show_dev_ptr_->disable();
        laser_show_dev_ptr_->disable();
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
  im_show_dev_ptr_->show_image(is_show_image_);
  laser_show_dev_ptr_->show_image(is_show_image_);
}

}  // namespace calibration
