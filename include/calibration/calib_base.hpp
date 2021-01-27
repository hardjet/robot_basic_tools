#pragma once

//  前向声明
namespace glk {
class GLSLShader;
}

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
  void show() { is_show_window_ = true; }

  template <typename T, typename C>
  void draw_sensor_selector(const std::string& name, dev::SENSOR_TYPE type, T &sensor);

  /**
   * @brief opengl渲染
   * @param shader
   */
  virtual void draw_gl(glk::GLSLShader& shader) = 0;

  /**
   * @brief imgui绘图
   */
  virtual void draw_ui() = 0;

 protected:
  // 是否显示ui窗口
  bool is_show_window_{false};
  // 传感器管理器
  std::shared_ptr<dev::SensorManager> sensor_manager_ptr_;
};
}  // namespace calibration