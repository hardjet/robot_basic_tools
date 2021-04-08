
#include "imgui.h"
#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

#include "dev/april_board.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/camera.hpp"
#include "dev/util.hpp"

#include "calibration/task_back_ground.hpp"
#include "calibration/two_cameras_calib.hpp"

// ---- 两个相机标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取相机位姿和落在标定板上的激光点
#define STATE_GET_POSE_AND_PTS 2
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
