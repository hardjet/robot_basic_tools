#include "imgui.h"

#include "dev/camera.hpp"
#include "dev/laser.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "calibration/camera_laser_calib.hpp"

namespace calibration {

CamLaserCalib::CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                             std::shared_ptr<dev::AprilBoard>& april_board_ptr_)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr_) {  // 图像显示
  im_show_ptr_ = std::make_shared<dev::ImageShow>();
};

void CamLaserCalib::draw_gl(glk::GLSLShader& shader) {}

void CamLaserCalib::draw_ui() {
  if (!is_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Camera and Laser Calibration", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 相机选择
  draw_sensor_selector<dev::Camera>("camera", dev::CAMERA, cam_ptr_);
  // 激光选择
  draw_sensor_selector<dev::Laser>("laser", dev::LASER, laser_ptr_);


  ImGui::End();
}

}  // namespace calibration
