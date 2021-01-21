#include "imgui.h"

#include "portable-file-dialogs.h"

// 传感器
#include "dev/camera.hpp"
#include "dev/sensor_manager.hpp"

namespace dev {

void SensorManager::add_sensor(dev::Sensor::Ptr &sensor) {
  //如果没有新建
  auto rs = sensors_map_.find(sensor->sensor_type);
  if (rs == sensors_map_.end()) {
    std::list<dev::Sensor::Ptr> sensor_list;
    sensor_list.push_back(sensor);
    sensors_map_[sensor->sensor_type] = sensor_list;
  } else {
    // 有就添加
    rs->second.push_back(sensor);
  }
}

void SensorManager::check_and_clear_sensors() {
  for (auto sensors_it = sensors_map_.begin(); sensors_it != sensors_map_.end();) {
    for (auto sensor_it = sensors_it->second.begin(); sensor_it != sensors_it->second.end();) {
      // 检查是否需要删除
      if ((*sensor_it)->is_to_be_deleted()) {
        sensor_it = sensors_it->second.erase(sensor_it);
      } else {
        ++sensor_it;
      }
    }
    // 检查当前类型的传感器是否为空
    if (sensors_it->second.empty()) {
      sensors_it = sensors_map_.erase(sensors_it);
    } else {
      ++sensors_it;
    }
  }
}

void SensorManager::draw_ui() {
  // 需要执行删除操作
  static bool need_to_delete = false;
  static int sensor_cnt = 0;
  int sensor_id = 0;

  // 设置窗口位置和大小
  ImGui::SetNextWindowPos(ImVec2(2, 22), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200, 400), ImGuiCond_FirstUseEver);

  ImGui::Begin("devices list", nullptr, ImGuiWindowFlags_None);
  if (ImGui::Button("Add +")) {
    ImGui::OpenPopup("add_devices_popup");
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("add a sensor by click the button.");
  }

  // 添加设备菜单
  if (ImGui::BeginPopup("add_devices_popup")) {
    ImGui::MenuItem("devices", nullptr, false, false);
    ImGui::Separator();
    if (ImGui::MenuItem("camera")) {
      std::string sensor_name_tmp = "new camere " + std::to_string(sensor_cnt);
      dev::Sensor::Ptr sensor = std::make_shared<dev::Camera>(sensor_name_tmp.c_str(), nh_);
      add_sensor(sensor);
      sensor_cnt++;
    }
    if (ImGui::MenuItem("laser")) {
    }
    ImGui::EndPopup();
  }

  // 分隔符
  ImGui::Separator();

  // // 用于高亮当前选择
  // static int selected = -1;
  // // 用于高亮当前选择
  // int select_cnt = 0;
  // 遍历设备 显示
  for (auto &sensors : sensors_map_) {
    // 显示设备类型的名称
    auto header_name = sensors.second.front()->sensor_type_str;
    if (ImGui::CollapsingHeader(header_name.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      for (auto &sensor : sensors.second) {
        // 高亮有问题 禁用
        // if (ImGui::Selectable("", selected == select_cnt)) {
        //   selected = select_cnt;
        // }
        // select_cnt++;
        // ImGui::SameLine();

        // 显示当前设备状态
        sensor->draw_status();
        ImGui::SameLine();
        // 显示设备名称
        ImGui::Text("%s", sensor->sensor_name.c_str());
        // 提示设备状态
        if (ImGui::IsItemHovered()) {
          if (sensor->is_sensor_online()) {
            ImGui::SetTooltip("%s online", sensor->sensor_name.c_str());
          } else {
            ImGui::SetTooltip("%s offline", sensor->sensor_name.c_str());
          }
        }

        // 编辑按钮
        ImGui::SameLine();
        ImGui::PushID(sensor_id);
        if (ImGui::SmallButton("edit")) {
          // 启动ui显示
          sensor->show();
        }
        ImGui::PopID();
        sensor_id++;

        // 删除按钮
        ImGui::SameLine();
        ImGui::PushID(sensor_id);
        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.9444, 1.0000, 0.6000));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.0,0.75,0.8));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.0,0.75,0.8));
        if (ImGui::SmallButton("del")) {
          std::string msg = "are you sure to delete [" + sensor->sensor_name + "] ?";
          pfd::message message("sensor delete", msg, pfd::choice::ok_cancel);
          while (!message.ready()) {
            usleep(100);
          }
          if (message.result() == pfd::button::ok) {
            // 标记为删除
            sensor->marked_to_be_deleted();
            need_to_delete = true;
          }
        }
        ImGui::PopStyleColor(3);
        ImGui::PopID();
        sensor_id++;
      }
    }
  }

  if (need_to_delete) {
    check_and_clear_sensors();
    need_to_delete = false;
  }

  if (sensors_map_.empty()) {
    ImGui::Text("please add sensor first.");
  }

  ImGui::End();
}

SensorManager::~SensorManager() { sensors_map_.clear(); }

}  // namespace dev