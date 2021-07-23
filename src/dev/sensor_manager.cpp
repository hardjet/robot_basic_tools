#include "imgui.h"
#include "portable-file-dialogs.h"

// 传感器
#include "dev/camera.hpp"
#include "dev/laser.hpp"
#include "dev/sensor_manager.hpp"

namespace dev {

void SensorManager::add_sensor(dev::Sensor::Ptr &sensor) {
  //如果没有新建
  auto rs = sensors_map.find(sensor->sensor_type);
  if (rs == sensors_map.end()) {
    std::list<dev::Sensor::Ptr> sensor_list;
    sensor_list.push_back(sensor);
    sensors_map[sensor->sensor_type] = sensor_list;
  } else {
    // 有就添加
    rs->second.push_back(sensor);
  }
}

void SensorManager::check_and_clear_sensors() {
  for (auto sensors_it = sensors_map.begin(); sensors_it != sensors_map.end();) {
    for (auto sensor_it = sensors_it->second.begin(); sensor_it != sensors_it->second.end();) {
      // 检查是否需要删除
      if ((*sensor_it)->b_to_be_deleted()) {
        sensor_it = sensors_it->second.erase(sensor_it);
      } else {
        ++sensor_it;
      }
    }
    // 检查当前类型的传感器是否为空
    if (sensors_it->second.empty()) {
      sensors_it = sensors_map.erase(sensors_it);
    } else {
      ++sensors_it;
    }
  }
}

void SensorManager::call_sensors_draw_ui() {
  for (auto &sensors : sensors_map) {
    for (auto &sensor : sensors.second) {
      sensor->draw_ui();
    }
  }
}

void SensorManager::draw_gl(glk::GLSLShader &shader, const std::shared_ptr<guik::GLCanvas>& canvas_ptr) {
  canvas_ptr->text_renderer_params.emplace_back("Sensor_Manager::draw_gl() - test", 20.0f, 20.0f, 0.3f, glm::vec3(1, 0, 0));
  for (auto &sensors : sensors_map) {
    for (auto &sensor : sensors.second) {
      sensor->draw_gl(shader, canvas_ptr);
    }
  }
}

void SensorManager::free() {
  for (auto &sensors : sensors_map) {
    for (auto &sensor : sensors.second) {
      sensor->free();
    }
  }
}

void SensorManager::draw_ui() {
  // 需要执行删除操作
  bool need_to_delete = false;
  int widgets_id = 0;

  // 设置窗口位置和大小
  ImGui::SetNextWindowPos(ImVec2(2, 22), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200, 400), ImGuiCond_FirstUseEver);

  ImGui::Begin("Devices", nullptr, ImGuiWindowFlags_None);

  if (ImGui::Button("Add +")) {
    ImGui::OpenPopup("add_devices_popup");
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("add a sensor by clicking the button.");
  }

  // 添加设备菜单
  if (ImGui::BeginPopup("add_devices_popup")) {
    ImGui::MenuItem("[device type]", nullptr, false, false);
    ImGui::Separator();
    if (ImGui::MenuItem("camera")) {
      std::string sensor_name_tmp = "new camere " + std::to_string(sensor_num_);
      dev::Sensor::Ptr sensor = std::make_shared<dev::Camera>(sensor_name_tmp.c_str(), nh_);
      add_sensor(sensor);
      sensor_num_++;
    }
    if (ImGui::MenuItem("laser")) {
      std::string sensor_name_tmp = "new laser " + std::to_string(sensor_num_);
      dev::Sensor::Ptr sensor = std::make_shared<dev::Laser>(sensor_name_tmp.c_str(), nh_);
      add_sensor(sensor);
      sensor_num_++;
    }
    ImGui::EndPopup();
  }

  // 分隔符
  ImGui::Separator();

  // // 用于高亮当前选择
  // static int selected = -1;
  // int select_cnt = 0;
  // 遍历设备 显示
  for (auto &sensors : sensors_map) {
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
        sensor->draw_status();        // 这里改了没反应？
        ImGui::SameLine();
        // 显示设备名称
        ImGui::Text("%s", sensor->sensor_name.c_str());
        // 提示设备状态
        if (ImGui::IsItemHovered()) {
          if (sensor->is_sensor_online()) {
            ImGui::SetTooltip("[%s] online", sensor->sensor_name.c_str());
          } else {
            ImGui::SetTooltip("[%s] offline", sensor->sensor_name.c_str());
          }
        }

        // 编辑按钮
        ImGui::SameLine();
        ImGui::PushID(widgets_id);
        if (ImGui::SmallButton("edit")) {
          // 启动ui显示
          sensor->show();
        }
        ImGui::PopID();
        widgets_id++;

        // 删除按钮
        ImGui::SameLine();
        ImGui::PushID(widgets_id);
        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.9444, 1.0000, 0.6000));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.0, 0.75, 0.8));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.0, 0.75, 0.8));
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
        widgets_id++;
      }
    }
  }

  // 清理标记删除的传感器
  if (need_to_delete) {
    check_and_clear_sensors();
    need_to_delete = false;
  }

  if (sensors_map.empty()) {
    ImGui::Text("please add sensor first.");
  }

  ImGui::End();

  // 显示传感器ui
  call_sensors_draw_ui();
}

SensorManager::~SensorManager() { sensors_map.clear(); }

}  // namespace dev