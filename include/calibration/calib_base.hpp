#pragma once

#include "imgui.h"

#include "dev/camera.hpp"
#include "dev/sensor.hpp"
#include "dev/sensor_manager.hpp"
#include "calibration/calibration_state.hpp"

//  前向声明
namespace glk {
class GLSLShader;
}  // namespace glk

namespace dev {
class SensorManager;
}  // namespace dev

namespace calibration {

class BaseCalib {
 public:
  explicit BaseCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr)
      : sensor_manager_ptr_(sensor_manager_ptr){};

  virtual ~BaseCalib() = default;

  /**
   * @brief 打开显示ui开关
   */
  void show() { b_show_window_ = true; }

  /**
   *
   * @tparam T 传感器Class
   * @param name 提示框文字
   * @param type 传感器类型
   * @param sensor 传感器指针
   */
  template <class T>
  void draw_sensor_selector(const std::string& name, dev::SENSOR_TYPE type, std::shared_ptr<T>& sensor) {
    // 保证控件中文字对齐
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s:", name.c_str());
    ImGui::SameLine();

    // 如果没有传感器对象，需要提示
    auto sensors_iter = sensor_manager_ptr_->sensors_map.find(type);
    if (sensors_iter == sensor_manager_ptr_->sensors_map.end()) {
      ImGui::TextColored(ImVec4{1.0, 0., 0., 1.}, "please add %s first!", dev::dev_type_str[type].c_str());
    } else {
      // 保存传感器对象名称
      std::vector<std::string> sensor_names;
      // 传感器ID
      int sensor_id;

      if (!sensor) {
        sensor = std::dynamic_pointer_cast<T>(sensors_iter->second.front());
        sensor_id = sensor->sensor_id;
      } else {
        sensor_id = sensor->sensor_id;
      }

      std::string comb_list_name = "##" + name + "_list";

      if (ImGui::BeginCombo(comb_list_name.c_str(), sensor->sensor_name.c_str(), ImGuiComboFlags_None)) {
        // 添加列表
        for (auto& c : sensors_iter->second) {
          sensor_names.push_back(c->sensor_name);
          // 判断是否为之前选择的
          bool is_selected = (sensor_id == c->sensor_id);
          // 是否按下
          if (ImGui::Selectable(sensor_names.back().c_str(), is_selected)) {
            // 更换需要重新赋值操作
            if (!is_selected) {
              sensor = std::dynamic_pointer_cast<T>(c);
              sensor_id = sensor->sensor_id;
            }
          }

          // 高亮之前选择的
          if (is_selected) {
            ImGui::SetItemDefaultFocus();
          }
        }

        ImGui::EndCombo();
      }
    }
  }

  /**
   * @brief opengl渲染
   * @param shader
   */
  virtual void draw_gl(glk::GLSLShader& shader) = 0;

  /**
   * @brief imgui绘图
   */
  virtual void draw_ui() = 0;

  /**
   * @brief 更新当前标定状态
   */
  virtual void change_current_state(std::shared_ptr<CalibrationState> new_state) = 0;

  /**
   * @brief 更新下个标定状态
   */
  virtual void change_next_state(std::shared_ptr<CalibrationState> new_state) = 0;

  /**
   * @brief 被标定的设备是否都有新数据
   */
  virtual bool instrument_available() = 0;

  virtual int pose_valid() = 0;

  virtual void check_steady() = 0;

  virtual int do_calib() = 0;

  std::shared_ptr<CalibrationState> next_state() { return next_state_ptr_; }

 protected:
  // 是否显示ui窗口
  bool b_show_window_{false};
  // 传感器管理器
  std::shared_ptr<dev::SensorManager> sensor_manager_ptr_;
  // 标定流程状态 当前
  int cur_state_{0};
  // 标定流程状态 下个
  int next_state_{0};
  // state reference to the current state of the context
  std::shared_ptr<CalibrationState> cur_state_ptr_;
  // state reference to the next state of the context
  std::shared_ptr<CalibrationState> next_state_ptr_;
};
}  // namespace calibration