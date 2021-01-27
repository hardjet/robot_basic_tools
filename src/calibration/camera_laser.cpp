#include "imgui.h"

#include "dev/camera.hpp"
#include "dev/sensor_manager.hpp"
#include "calibration/camera_laser.hpp"

namespace calibration {

void CamLaserCalib::draw_gl(glk::GLSLShader& shader) {}

void CamLaserCalib::draw_ui() {
  if (!is_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Camera and Laser Calibration", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  draw_sensor_selector<dev::Camera::Ptr, dev::Camera>("camera", dev::CAMERA, cam_ptr_);

  // // 保证控件中文字对齐
  // ImGui::AlignTextToFramePadding();
  // ImGui::Text("camera:");
  // ImGui::SameLine();
  //
  // // 如果没有相机对象，需要提示
  // auto cam = sensor_manager_ptr_->sensors_map.find(dev::SENSOR_TYPE::CAMERA);
  // if (cam == sensor_manager_ptr_->sensors_map.end()) {
  //   ImGui::TextColored(ImVec4{1.0, 0., 0., 1.}, "please add camera first!");
  // } else {
  //   // 保存相机对象名称
  //   std::vector<std::string> cam_names;
  //   // 传感器ID
  //   int cam_id;
  //
  //   if (!cam_ptr_) {
  //     cam_ptr_ = std::dynamic_pointer_cast<dev::Camera>(cam->second.front());
  //     cam_id = cam_ptr_->sensor_id;
  //   } else {
  //     cam_id = cam_ptr_->sensor_id;
  //   }
  //
  //   if (ImGui::BeginCombo("##cam_list", cam_ptr_->sensor_name.c_str(), ImGuiComboFlags_None)) {
  //     // 添加列表
  //     for (auto& c : cam->second) {
  //       cam_names.push_back(c->sensor_name);
  //       // 判断是否为之前选择的
  //       bool is_selected = (cam_id == c->sensor_id);
  //       // 是否按下
  //       if (ImGui::Selectable(cam_names.back().c_str(), is_selected)) {
  //         // 更换需要重新赋值操作
  //         if (!is_selected) {
  //           cam_ptr_ = std::dynamic_pointer_cast<dev::Camera>(c);
  //           cam_id = cam_ptr_->sensor_id;
  //         }
  //       }
  //
  //       // 高亮之前选择的
  //       if (is_selected) {
  //         ImGui::SetItemDefaultFocus();
  //       }
  //     }
  //
  //     ImGui::EndCombo();
  //   }
  // }

  // // 保证控件中文字对齐
  // ImGui::AlignTextToFramePadding();
  // ImGui::Text("laser:");
  //
  // // 如果没有laser对象，需要提示
  // auto laser = sensor_manager_ptr_->sensors_map.find(dev::SENSOR_TYPE::LASER);
  // if (laser == sensor_manager_ptr_->sensors_map.end()) {
  //   ImGui::TextColored(ImVec4{1.0, 0., 0., 1.}, "please add laser first!");
  // } else {
  //   // 保存相机对象名称
  //   std::vector<std::string> laser_names;
  //   // 传感器ID
  //   int laser_id;
  //
  //   if (!cam_ptr_) {
  //     cam_ptr_ = std::dynamic_pointer_cast<dev::Camera>(cam->second.front());
  //     cam_id = cam_ptr_->sensor_id;
  //   } else {
  //     cam_id = cam_ptr_->sensor_id;
  //   }
  //
  //   if (ImGui::BeginCombo("##cam_list", cam_ptr_->sensor_name.c_str(), ImGuiComboFlags_None)) {
  //     // 添加列表
  //     for (auto& c : cam->second) {
  //       cam_names.push_back(c->sensor_name);
  //       // 判断是否为之前选择的
  //       bool is_selected = (cam_id == c->sensor_id);
  //       // 是否按下
  //       if (ImGui::Selectable(cam_names.back().c_str(), is_selected)) {
  //         // 更换需要重新赋值操作
  //         if (!is_selected) {
  //           cam_ptr_ = std::dynamic_pointer_cast<dev::Camera>(c);
  //           cam_id = cam_ptr_->sensor_id;
  //         }
  //       }
  //
  //       // 高亮之前选择的
  //       if (is_selected) {
  //         ImGui::SetItemDefaultFocus();
  //       }
  //     }
  //
  //     ImGui::EndCombo();
  //   }
  // }

  ImGui::End();
}

}  // namespace calibration
