#pragma once

#include <memory>
#include "calib_base.hpp"

namespace dev {
class AprilBoard;
class Camera;
class Laser;
class ImageShow;
}  // namespace dev

namespace calibration {

class CamLaserCalib : public BaseCalib {
 public:
  CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                std::shared_ptr<dev::AprilBoard>& april_board_ptr_);

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;

 private:
  // 标定板
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 当前选中的相机对象
  std::shared_ptr<dev::Camera> cam_ptr_{nullptr};
  // 当前选中的相机对象
  std::shared_ptr<dev::Laser> laser_ptr_{nullptr};
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> im_show_ptr_{nullptr};
};
}  // namespace calibration
