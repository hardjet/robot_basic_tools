#include <iostream>

#include "imgui.h"
#include "portable-file-dialogs.h"

#include "dev/sensor_data.hpp"
#include "dev/laser.hpp"
#include "dev/util.hpp"

namespace dev {

Laser::Laser(const std::string& name, ros::NodeHandle& ros_nh) : Sensor(name, ros_nh, SENSOR_TYPE::LASER, "LASER") {
  sensor_topic_list_.resize(1);

  // 设置深度点云数据接收
  laser_data_ = std::make_shared<SensorData<sensor_msgs::LaserScan>>(nh_, 5);
  // 频率/2
  laser_data_->set_data_rate(2);
}

boost::shared_ptr<const sensor_msgs::LaserScan> Laser::data(){return laser_data_->data();}

void Laser::draw_gl(glk::GLSLShader& shader) {}

void Laser::check_online_status() {
  bool online = false;

  // 获取深度点云最新数据
  auto laser_data_ptr = laser_data_->data();
  if (laser_data_ptr) {
    if (ros::Time::now().sec - laser_data_ptr->header.stamp.sec < 2) {
      online = true;
    }
  }

  is_online_ = online;
}

void Laser::draw_ui_topic_name() {
  // 名称控件变量
  char laser_topic_name_char[128]{""};

  memcpy(laser_topic_name_char, sensor_topic_list_[0].c_str(), sensor_topic_list_[0].length());

  ImGui::Separator();

  if (ImGui::Checkbox("##laser_topic", &enable_topic_)) {
    // 选中状态
    if (enable_topic_) {
      // 如果topic有效才启用数据接收
      if (sensor_topic_list_[0].empty()) {
        show_pfd_info("warning", "please set topic name first!");
        enable_topic_ = false;
      } else {
        laser_data_->subscribe(sensor_topic_list_[0], 5);
      }
    } else {
      // 暂停接收
      laser_data_->unsubscribe();
    }
  }
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("pick to enable laser receive");
  }
  ImGui::SameLine();
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("laser topic:");
  ImGui::SameLine();
  // ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.86f);
  // 只有按回车才保存
  if (ImGui::InputTextWithHint("##laser_topic_name", "press 'ENTER' to save", laser_topic_name_char, 128,
                               ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (laser_topic_name_char[0] != '\0' && laser_topic_name_char[0] != ' ') {
      sensor_topic_list_[0] = laser_topic_name_char;
      // 暂停接收
      laser_data_->unsubscribe();
      enable_topic_ = false;
    }
  } else {
    // 恢复名称
    memset(laser_topic_name_char, 0, 128);
    memcpy(laser_topic_name_char, sensor_topic_list_[0].c_str(), sensor_topic_list_[0].length());
  }
  ImGui::SameLine();
  // 从ros系统中选择话题
  int selected_id = -1;
  if (ImGui::Button("...##laser")) {
    get_topic_name_from_list("sensor_msgs/LaserScan", ros_topic_selector_);
    ImGui::OpenPopup("popup##laser");
  }
  if (ImGui::BeginPopup("popup##laser")) {
    if (ros_topic_selector_.empty()) {
      ImGui::Text("no sensor_msgs/LaserScan msgs available.");
    } else {
      ImGui::Text("[sensor_msgs/LaserScan list]");
      ImGui::Separator();
      for (int i = 0; i < ros_topic_selector_.size(); i++) {
        // 按下
        if (ImGui::Selectable(ros_topic_selector_[i].c_str())) {
          selected_id = i;
        }
      }
    }
    // 变更话题名称
    if (selected_id != -1) {
      sensor_topic_list_[0] = ros_topic_selector_[selected_id];
      // 暂停接收
      laser_data_->unsubscribe();
      enable_topic_ = false;
    }

    ImGui::EndPopup();
  }
}

void Laser::draw_ui() {
  // 名称控件变量
  char name_char[128]{""};

  if (!is_show_window_) {
    return;
  }

  // 初始化为设备名称
  memcpy(name_char, sensor_name.c_str(), sensor_name.length());

  // 保证窗口名称唯一
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
    memset(name_char, 0, 128);
    memcpy(name_char, sensor_name.c_str(), sensor_name.length());
  }

  draw_ui_topic_name();

  ImGui::End();

  // 检查设备在线状态
  check_online_status();
}

}  // namespace dev