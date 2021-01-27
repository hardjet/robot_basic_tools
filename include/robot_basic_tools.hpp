#include <memory>

#include "guik/imgui_application.hpp"

namespace dev {
class SensorManager;
class AprilBoard;
}

namespace calibration {
class CamLaserCalib;
}



class RobotBasicTools : public guik::Application {
 public:
  explicit RobotBasicTools(ros::NodeHandle &ros_nh) : Application(), nh(ros_nh) {}

  ~RobotBasicTools() override = default;

  /**
   * @brief initialize the application
   * @param window_name
   * @param size
   * @param glsl_version
   * @return true
   * @return false
   */
  bool init(const char *window_name, const char *imgui_config_path, const Eigen::Vector2i &size,
            const char *glsl_version) override;

  /**
   * @brief draw ImGui-based UI
   *
   */
  void draw_ui() override;

  /**
   * @brief draw OpenGL related stuffs on the main canvas
   *
   */
  void draw_gl() override;

  /**
   * @brief frame buffer size change callback
   *
   * @param size
   */
  void framebuffer_size_callback(const Eigen::Vector2i &size) override { main_canvas_ptr->set_size(size); }

 private:
  /**
   * @brief draw main menu
   *
   */
  void main_menu();

 private:
  /**
   * @brief handling mouse input
   */
  void mouse_control();

  /**
   * @brief context menu
   */
  void context_menu();

 private:
  // ros nodehandle
  ros::NodeHandle &nh;
  // 鼠标右击位置
  Eigen::Vector2i right_clicked_pos;
  // 当前鼠标位置
  Eigen::Vector2i cur_mouse_pos;

  // 主画布
  std::unique_ptr<guik::GLCanvas> main_canvas_ptr;
  // 进度条
  std::unique_ptr<guik::ProgressModal> progress_ptr;

  // 热键标记(ALT)
  bool is_hotkey_alt_pressed = false;
  // 选中的物体集合
  // std::set<int> selected_id;

  // ui显示标记
  bool is_show_imgui_demo{false};

  // 传感器管理器
  std::shared_ptr<dev::SensorManager> sensor_manager_ptr;

  // 标定板
  std::shared_ptr<dev::AprilBoard> april_board_ptr;

  // 相机与单线激光标定
  std::shared_ptr<calibration::CamLaserCalib> cl_calib_ptr;
};
