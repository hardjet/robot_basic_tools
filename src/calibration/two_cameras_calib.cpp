#include <cv_bridge/cv_bridge.h>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

#include "dev/april_board.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/camera.hpp"
#include "dev/util.hpp"

#include "algorithm/util.h"

#include "calibration/task_back_ground.hpp"
#include "calibration/two_cameras_calib.hpp"

#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/camera_models/Camera.h"

// ---- 两个相机标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取相机位姿
#define STATE_GET_POSE 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3
// 开始标定
#define STATE_START_CALIB 4
// 在标定过程中
#define STATE_IN_CALIB 5

namespace calibration {

TwoCamerasCalib::TwoCamerasCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                                 std::shared_ptr<dev::AprilBoard>& april_board_ptr)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr) {
  // 设置id
  camera_insts_[0].id = 0;
  camera_insts_[1].id = 1;

  // 图像显示控件
  camera_insts_[0].img_show_dev_ptr = std::make_shared<dev::ImageShow>();
  camera_insts_[1].img_show_dev_ptr = std::make_shared<dev::ImageShow>();

  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
  // 设置相对位姿初始值
  T_12_ = Eigen::Matrix4f::Identity();
}

void TwoCamerasCalib::update() {
  std::lock_guard<std::mutex> lock(mtx_);

  // 更新数据
  for (auto& camera_inst : camera_insts_) {
    if (!camera_inst.cv_image_data_ptr) {
      camera_inst.cv_image_data_ptr = camera_inst.camera_dev_ptr->data();
      if (camera_inst.cv_image_data_ptr) {
        camera_inst.is_new_data = true;
      }
    } else {
      auto camera_data = camera_inst.camera_dev_ptr->data();
      if (camera_inst.cv_image_data_ptr->header.stamp.nsec != camera_data->header.stamp.nsec) {
        camera_inst.cv_image_data_ptr = camera_data;
        camera_inst.is_new_data = true;
      }
    }
  }
}

bool TwoCamerasCalib::get_valid_pose() {
  // 当前处理的图像数据
  std::vector<boost::shared_ptr<const cv_bridge::CvImage>> cur_images(2);

  auto start_time = ros::WallTime::now();
  // 将图像锁定
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < 2; i++) {
      auto& cv_image_ptr = camera_insts_[i].cv_image_data_ptr;
      cur_images[i].reset(
          new const cv_bridge::CvImage(cv_image_ptr->header, cv_image_ptr->encoding, cv_image_ptr->image));
    }
  }

  for (int i = 0; i < 2; i++) {
    auto& camera_inst = camera_insts_[i];
    cv::Mat img;
    // 需要转为灰度
    if (cur_images[i]->image.channels() == 3) {
      cv::cvtColor(cur_images[i]->image, img, cv::COLOR_RGB2GRAY);
    } else {
      img = cur_images[i]->image.clone();
    }

    cv::Mat img_show;
    // 需要转为彩色图像
    if (cur_images[i]->image.channels() == 1) {
      cv::cvtColor(camera_inst.cv_image_data_ptr->image, img_show, cv::COLOR_GRAY2RGB);
    } else {
      img_show = camera_inst.cv_image_data_ptr->image.clone();
    }

    if (april_board_ptr_->board->computeObservation(img, img_show, camera_inst.object_points,
                                                    camera_inst.image_points)) {
      // 计算外参T_ac camera_model->aprilboard
      camera_inst.camera_dev_ptr->camera_model()->estimateExtrinsics(
          camera_inst.object_points, camera_inst.image_points, camera_inst.T_ac, img_show);
      // calib_data_.timestamp = cur_image->header.stamp.toSec();
      // calib_data_.t_ac = T_ac.block(0, 3, 3, 1);
      // Eigen::Matrix3d R_ac = T_ac.block(0, 0, 3, 3);
      // Eigen::Quaterniond q_ac(R_ac);
      // calib_data_.q_ac = q_ac;

      // 保存
      camera_inst.show_cv_image_ptr.reset(
          new const cv_bridge::CvImage(cur_images[i]->header, cur_images[i]->encoding, img_show));
      // 更新显示图象
      camera_inst.update();

    } else {
      // std::cout << "no april board detected!" << std::endl;
      return false;
    }
  }

  return true;
}

void TwoCamerasCalib::calibration() {
  // 首先获取最新的数据
  update();

  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      // 都有新数据才计算
      if (camera_insts_[0].is_new_data && camera_insts_[1].is_new_data) {
        camera_insts_[0].is_new_data = false;
        camera_insts_[1].is_new_data = false;
        cur_state_ = STATE_GET_POSE;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_POSE:
      // 执行任务
      if (task_ptr_->do_task("get_pose_and_points", std::bind(&TwoCamerasCalib::get_valid_pose, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          cur_state_ = STATE_CHECK_STEADY;
        } else {
          cur_state_ = STATE_IDLE;
        }
      }
      break;
    case STATE_CHECK_STEADY:
      if (calib_data_vec_.empty()) {
        calib_data_vec_.push_back(calib_data_);
      } else {
        // 检查相机位姿是否一致
        double delta = abs(calib_data_vec_.at(0).angle - calib_data_.angle);
        std::cout << "delta: " << delta << " deg" << std::endl;
        // 抖动小于0.2°
        if (delta < 0.2) {
          calib_data_vec_.push_back(calib_data_);
          // 足够稳定才保存
          if (calib_data_vec_.size() > 6) {
            check_and_save();
          }
        } else {
          // 抖动大则重新开始检测
          calib_data_vec_.clear();
          calib_data_vec_.push_back(calib_data_);
          std::cout << "moved!!!" << std::endl;
        }
      }
      cur_state_ = STATE_IDLE;
      break;
    case STATE_START_CALIB:
      cur_state_ = STATE_IN_CALIB;
      break;
    case STATE_IN_CALIB:
      // 执行任务
      if (task_ptr_->do_task("calc", std::bind(&TwoCamerasCalib::calc, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          // 将计算结果更新到laser2
          update_camera2_pose();
          update_ui_transform();
          cur_state_ = STATE_IDLE;
        }
      }
      break;
  }
}

void TwoCamerasCalib::draw_calib_params() {
  const double min_v = 0.;
  ImGui::Separator();
  ImGui::Text("calibration params:");

  ImGui::DragScalar("between angle(deg)", ImGuiDataType_Double, &between_angle_, 1.0, &min_v, nullptr, "%.1f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("set angle between valid candidate.");
  }

  ImGui::PopItemWidth();
}

void TwoCamerasCalib::draw_ui_transform() {
  ImGui::Separator();
  ImGui::Text("T_12:");
  ImGui::SameLine();
  ImGui::TextDisabled("the unit: m & deg");
  ImGui::Text("set the transform from camera 2 to 1.");

  // 变更后要更新矩阵
  bool is_changed = false;
  // 设定宽度
  ImGui::PushItemWidth(80);
  if (ImGui::DragScalar("tx  ", ImGuiDataType_Float, &transform_12_[0], 0.1, nullptr, nullptr, "%.3f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("ty   ", ImGuiDataType_Float, &transform_12_[1], 0.1, nullptr, nullptr, "%.3f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("tz", ImGuiDataType_Float, &transform_12_[2], 0.1, nullptr, nullptr, "%.3f")) {
    is_changed = true;
  }

  if (ImGui::DragScalar("roll", ImGuiDataType_Float, &transform_12_[3], 1.0, nullptr, nullptr, "%.2f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("pitch", ImGuiDataType_Float, &transform_12_[4], 1.0, nullptr, nullptr, "%.2f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("yaw", ImGuiDataType_Float, &transform_12_[5], 1.0, nullptr, nullptr, "%.2f")) {
    is_changed = true;
  }

  ImGui::PopItemWidth();
  // 更新变换矩阵
  if (is_changed) {
    Eigen::Quaterniond q_12 = algorithm::ypr2quaternion(DEG2RAD_RBT(transform_12_[5]), DEG2RAD_RBT(transform_12_[4]),
                                                        DEG2RAD_RBT(transform_12_[3]));
    Eigen::Vector3f t_12{transform_12_[0], transform_12_[1], transform_12_[2]};
    // 更新变换矩阵
    T_12_.block<3, 3>(0, 0) = q_12.toRotationMatrix().cast<float>();
    T_12_.block<3, 1>(0, 3) = t_12;
    // std::cout << "T12:\n" << T_12_ << std::endl;

    // update_laser2_pose();
  }

  ImGui::Separator();
}

void TwoCamerasCalib::draw_ui() {
  if (!b_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Two Cameras Calibration", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 相机选择
  draw_sensor_selector<dev::Camera>("camera 1", dev::CAMERA, camera_insts_[0].camera_dev_ptr);

  // 从文件加载标定数据
  ImGui::SameLine();
  if (ImGui::Button("R")) {
    // 选择加载文件路径
    std::vector<std::string> filters = {"calib data file (.json)", "*.json"};
    std::unique_ptr<pfd::open_file> dialog(new pfd::open_file("choose file", dev::data_default_path, filters));
    while (!dialog->ready()) {
      usleep(1000);
    }

    auto file_paths = dialog->result();
    if (!file_paths.empty()) {
      // 从文件读数据
      if (load_calib_data(file_paths.front())) {
        std::string msg = "load calib data from " + file_paths.front() + " ok!";
        dev::show_pfd_info("load calib data", msg);
      } else {
        std::string msg = "load calib data from " + file_paths.front() + " failed!";
        dev::show_pfd_info("load calib data", msg);
      }
    }
  }
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("load calib data from .json file");
  }

  if (!calib_valid_data_vec_.empty()) {
    ImGui::SameLine();
    // 保存标定数据
    if (ImGui::Button("W")) {
      // 选择保存文件路径
      std::vector<std::string> filters = {"calib data file (.json)", "*.json"};
      std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", dev::data_default_path, filters));
      while (!dialog->ready()) {
        usleep(1000);
      }

      auto file_path = dialog->result();
      if (!file_path.empty()) {
        // 导出标定数据
        if (!save_calib_data(file_path)) {
          std::string msg = "save calib data to " + file_path + " failed!";
          dev::show_pfd_info("save calib data", msg);
        } else {
          std::string msg = "save calib data to " + file_path + " ok!";
          dev::show_pfd_info("save calib data", msg);
        }
      }
    }

    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("save calib data to .json file");
    }
  }

  // 相机选择
  draw_sensor_selector<dev::Camera>("camera 2", dev::CAMERA, camera_insts_[1].camera_dev_ptr);

  // 设备就绪后才能标定
  if (camera_insts_[0].camera_dev_ptr && camera_insts_[1].camera_dev_ptr) {
    ImGui::SameLine();
    // 选择是否显示图像
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &b_show_image_)) {
      if (b_show_image_) {
        camera_insts_[0].img_show_dev_ptr->enable("tc calib: camera 1", false);
        camera_insts_[1].img_show_dev_ptr->enable("tc calib: camera 2", false);
      } else {
        camera_insts_[0].img_show_dev_ptr->disable();
        camera_insts_[1].img_show_dev_ptr->disable();
      }
    }

    // 闲置状态下才可以设置
    if (next_state_ == STATE_IDLE) {
      draw_calib_params();
    }

    // 设置变换矩阵参数
    draw_ui_transform();

    // 标定逻辑
    calibration();

    if (next_state_ == STATE_IDLE) {
      if (ImGui::Button("start")) {
        // 检测相机模型是否已经选择
        if (!camera_insts_[0].camera_dev_ptr->camera_model() || !camera_insts_[1].camera_dev_ptr->camera_model()) {
          std::string msg = "please set camera model first!";
          dev::show_pfd_info("info", msg);
        } else {
          b_show_calib_data_ = false;
          // 清空上次的标定数据
          calib_valid_data_vec_.clear();
          next_state_ = STATE_START;
        }
      }
    } else {
      if (ImGui::Button("stop")) {
        next_state_ = STATE_IDLE;
      }
    }

    // 标定状态只需要设定一次
    if (cur_state_ == STATE_IN_CALIB) {
      next_state_ = STATE_IDLE;
    }

    // 有一帧数据就可以开始进行标定操作了
    if (!calib_valid_data_vec_.empty()) {
      ImGui::SameLine();
      // 开始执行标定
      if (ImGui::Button("calib")) {
        next_state_ = STATE_START_CALIB;
      }
    }
  }

  // 标定数据相关操作
  if (next_state_ == STATE_IDLE && !calib_valid_data_vec_.empty()) {
    if (ImGui::Checkbox("##show_calib_data", &b_show_calib_data_)) {
      if (b_show_calib_data_) {
        b_need_to_update_cd_ = true;
        // 显示AprilBoard
        april_board_ptr_->show_3d();
      }
    }
    if (ImGui::IsItemHovered()) {
      if (b_show_calib_data_) {
        ImGui::SetTooltip("click to not show calib data");
      } else {
        ImGui::SetTooltip("click to show calib data");
      }
    }

    float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    ImGui::PushButtonRepeat(true);
    ImGui::SameLine(0.0f, spacing);
    if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
      if (selected_calib_data_id_ > 1) {
        selected_calib_data_id_--;
        if (b_show_calib_data_) {
          b_need_to_update_cd_ = true;
        }
      }
    }
    ImGui::SameLine(0.0f, spacing);
    if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
      if (selected_calib_data_id_ < calib_valid_data_vec_.size()) {
        selected_calib_data_id_++;
        if (b_show_calib_data_) {
          b_need_to_update_cd_ = true;
        }
      }
    }
    ImGui::PopButtonRepeat();
    ImGui::SameLine();
    // 删除按钮
    if (!calib_valid_data_vec_.empty()) {
      if (ImGui::Button("Del")) {
        selected_calib_data_id_--;
        // 删除数据
        calib_valid_data_vec_.erase(calib_valid_data_vec_.begin() + selected_calib_data_id_);
        b_need_to_update_cd_ = true;
        if (selected_calib_data_id_ == 0) {
          selected_calib_data_id_ = 1;
        }
      }
      // tips
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("delete one item of calib data");
      }
    }

    ImGui::SameLine();
    ImGui::TextDisabled("vaild: %d/%zu", selected_calib_data_id_, calib_valid_data_vec_.size());
  } else if (camera_insts_[0].camera_dev_ptr && camera_insts_[1].camera_dev_ptr) {
    ImGui::SameLine();
    ImGui::TextDisabled("vaild: %zu", calib_valid_data_vec_.size());
  }

  ImGui::End();

  // 显示图像
  if (b_show_image_) {
    camera_insts_[0].img_show_dev_ptr->show_image(b_show_image_);
    camera_insts_[1].img_show_dev_ptr->show_image(b_show_image_);
  }
}

}  // namespace calibration
