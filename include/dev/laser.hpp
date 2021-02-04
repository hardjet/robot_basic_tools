#pragma once

#include <sensor_msgs/LaserScan.h>
#include "dev/sensor.hpp"

namespace dev {

template <class M>
class SensorData;

class ImageShow;

/**
 * @brief Laser object
 *
 */
class Laser : public Sensor {
 public:
  using Ptr = std::shared_ptr<Laser>;

  Laser(const std::string& name, ros::NodeHandle& ros_nh);

  ~Laser() override = default;

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;
  /// 获取当前激光数据
  boost::shared_ptr<const sensor_msgs::LaserScan> data();

 private:
  // 是否显示激光
  bool is_show_laser_{false};
  // ros topic 使能
  bool enable_topic_{false};
  // 点云数据
  std::shared_ptr<SensorData<sensor_msgs::LaserScan>> laser_data_;

 private:
  /// 绘制话题ui
  void draw_ui_topic_name();
  /// 检查当前设备在线状态
  void check_online_status();
};

}  // namespace dev
