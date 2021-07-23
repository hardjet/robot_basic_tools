#include <memory>

#include "util/tf_tree.hpp"
#include "guik/imgui_application.hpp"

namespace dev {
class SensorManager;
class AprilBoard;
class chessboard;
}  // namespace dev

namespace calibration {
class CamLaserCalib;
class TwoLasersCalib;
class TwoCamerasCalib;
class CameraCalib;
}

class RobotBasicTools : public guik::Application {
 public:
  explicit RobotBasicTools(ros::NodeHandle &ros_nh) : Application(), nh_(ros_nh) {}

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
  void framebuffer_size_callback(const Eigen::Vector2i &size) override { main_canvas_ptr_->set_size(size); }

  void free() override;

 private:
  /**
   * @brief draw main menu
   *
   */
  void main_menu();

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
  ros::NodeHandle &nh_;
  // 鼠标右击位置
  Eigen::Vector2i right_clicked_pos_;
  // 当前鼠标位置
  Eigen::Vector2i cur_mouse_pos_;

  // 主画布
  std::shared_ptr<guik::GLCanvas> main_canvas_ptr_;

  // 进度条
  std::unique_ptr<guik::ProgressModal> progress_ptr_;

  // 传感器管理器
  std::shared_ptr<dev::SensorManager> sensor_manager_ptr_;

  // APRILTAG标定板
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;

  // 标准棋盘格标定板
  std::shared_ptr<dev::chessboard> chess_board_ptr_;

  // 标准圆形标定板
  std::shared_ptr<dev::blob_board> blob_board_ptr_;

  // 相机与单线激光标定
  std::unique_ptr<calibration::CamLaserCalib> cl_calib_ptr_;

  // 两个单线激光标定
  std::unique_ptr<calibration::TwoLasersCalib> tl_calib_ptr_;

  // 两个相机标定
  std::unique_ptr<calibration::TwoCamerasCalib> tc_calib_ptr_;

  // 单目相机标定
  std::unique_ptr<calibration::CameraCalib> cam_calib_ptr_;

  // tf树
  std::unique_ptr<util::TfTree> tf_tree_ptr_;

  // 热键标记(ALT)
  bool b_hotkey_alt_pressed_ = false;
  // 选中的物体集合
  // std::set<int> selected_id;

  // ui显示标记
  bool b_show_imgui_demo_{false};
};
