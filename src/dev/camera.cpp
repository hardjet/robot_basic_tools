#include <iostream>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "dev/camera.hpp"
#include "camera_model/camera_models/CameraFactory.h"

namespace dev {

Camera::Camera(const std::string& name, ros::NodeHandle& ros_nh)
    : Sensor(name, ros_nh, SENSOR_TYPE::CAMERA, "CAMERA") {}

void Camera::draw_gl(glk::GLSLShader& shader) {}

void Camera::creat_instance(int current_camera_type) {
  inst_ptr_ = nullptr;
  switch (current_camera_type) {
    case camera_model::Camera::ModelType::KANNALA_BRANDT:
      inst_ptr_ = camera_model::CameraFactory::instance()->generateCamera(
          camera_model::Camera::ModelType::KANNALA_BRANDT, sensor_name, cv::Size{640, 480});
      break;
    case camera_model::Camera::ModelType::MEI:
      inst_ptr_ = camera_model::CameraFactory::instance()->generateCamera(camera_model::Camera::ModelType::MEI,
                                                                          sensor_name, cv::Size{640, 480});
      break;
    case camera_model::Camera::ModelType::PINHOLE:
      inst_ptr_ = camera_model::CameraFactory::instance()->generateCamera(camera_model::Camera::ModelType::PINHOLE,
                                                                          sensor_name, cv::Size{640, 480});
      break;
    default:
      break;
  }
  inst_ptr_->writeParameters(inst_params_);
}

void Camera::draw_ui_parms() {
  if (!inst_ptr_) {
    return;
  }

  switch (inst_ptr_->modelType()) {
    case camera_model::Camera::ModelType::KANNALA_BRANDT:
      break;
    case camera_model::Camera::ModelType::MEI:
      break;
    case camera_model::Camera::ModelType::PINHOLE:
      ImGui::Text("cx:");
      ImGui::SameLine();
      ImGui::InputDouble("##cx", &inst_params_[0], 1.0f, 10.f, "%.6f");
      ImGui::SameLine();
      ImGui::Text("cy:");
      ImGui::SameLine();
      ImGui::InputDouble("##cy", &inst_params_[2], 1.0f, 10.f, "%.6f");
      break;
    default:
      break;
  }
}

void Camera::draw_ui() {
  // 名称控件变量
  static char name_char[128]{" "};
  // 相机类型控件变量
  const char* camera_type[] = {"KANNALA_BRANDT", "MEI", "PINHOLE"};
  static int current_camera_type = 2;

  if (!is_show_window_) {
    return;
  }

  // 初始化为设备名称
  if (name_char[0] == ' ') {
    memcpy(name_char, sensor_name.c_str(), sensor_name.length());
  }

  // 保证窗口名称唯一
  ImGui::SetNextWindowPos(ImVec2(300, 22), ImGuiCond_FirstUseEver);

  std::string win_name = sensor_name + "##" + std::to_string(sensor_id);
  ImGui::Begin(win_name.c_str(), &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();

  // 修改名称
  ImGui::Text("sensor name:");
  ImGui::SameLine();
  // 只有按回车才保存
  if (ImGui::InputTextWithHint("##sensor name", "press 'ENTER' to save", name_char, IM_ARRAYSIZE(name_char),
                               ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (name_char[0] != '\0' && name_char[0] != ' ') {
      sensor_name = name_char;
    }
  } else {
    // 恢复名称
    memcpy(name_char, sensor_name.c_str(), sensor_name.length());
  }
  ImGui::SameLine();
  // 从文件加载相机
  if (ImGui::Button("R")) {
  }
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("load from .yaml");
  }

  // 将相机参数写入文件
  if (inst_ptr_) {
    ImGui::SameLine();
    if (ImGui::Button("W")) {
      // 选择保存文件路径
      std::vector<std::string> filters = {"camera config file (.yaml)", "*.yaml"};
      std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", "", filters));
      while (!dialog->ready()) {
        usleep(1000);
      }

      auto file_path = dialog->result();
      if (!file_path.empty()) {
        // 导出参数
        inst_ptr_->writeParametersToYamlFile(file_path);
      }
    }

    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("save to .yaml");
    }
  }

  // 相机类型控制
  ImGui::Text("camera type:");
  ImGui::SameLine();

  if (ImGui::Combo("##camera type", &current_camera_type, camera_type, IM_ARRAYSIZE(camera_type))) {
    // 如果状态改变 询问改变
    if (inst_ptr_) {
      std::string msg = "are you sure to change camera type ?";
      pfd::message message("type change", msg, pfd::choice::ok_cancel);
      while (!message.ready()) {
        usleep(1000);
      }
      // 新建实例
      if (message.result() == pfd::button::ok) {
        creat_instance(current_camera_type);
      } else {
        current_camera_type = inst_ptr_->modelType();
      }
    }
  }

  // 如果没有camera对象，可以选择新建
  if (!inst_ptr_) {
    ImGui::SameLine();
    if (ImGui::Button("new")) {
      creat_instance(current_camera_type);
    }
    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("create a new type camera");
    }
  }

  // 展示当前相机参数
  draw_ui_parms();

  ImGui::End();
}

}  // namespace dev