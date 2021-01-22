#pragma once

#include "dev/sensor.hpp"
#include <boost/shared_ptr.hpp>

namespace camera_model {
class Camera;
}

namespace dev {

/**
 * @brief Camera object
 *
 */
class Camera : public Sensor {
 public:
  using Ptr = std::shared_ptr<Camera>;

  Camera(const std::string& name, ros::NodeHandle& ros_nh);

  ~Camera() override = default;

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;

 private:
  boost::shared_ptr<camera_model::Camera> inst_ptr_{nullptr};
  std::vector<double> inst_params_;

 private:
  /// 创建指定类型的相机
  void creat_instance(int current_camera_type);
  /// 绘制参数ui
  void draw_ui_parms();
  /// 绘制话题ui
  void draw_ui_topic_name();
};

}  // namespace dev
