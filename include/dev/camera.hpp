#pragma once

#include "dev/sensor.hpp"

namespace dev {

/**
 * @brief Camera object
 *
 */
class Camera : public Sensor {
 public:
  using Ptr = std::shared_ptr<Camera>;

  Camera(const std::string& name, ros::NodeHandle& ros_nh) : Sensor(name, ros_nh, SENSOR_TYPE::CAMERA, "CAMERA"){
  }

  ~Camera() override = default;

  // opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  // imgui绘图
  void draw_ul() override;

};

}  // namespace dev
