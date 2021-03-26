#pragma once

#include <sensor_msgs/LaserScan.h>
#include "dev/sensor.hpp"

namespace glk {
class PointCloudBuffer;
}

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

  void free() override;

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;
  /// 获取当前激光数据
  boost::shared_ptr<const sensor_msgs::LaserScan> data();

 private:
  // 是否显示激光
  bool b_show_laser_{false};
  // ros topic 使能
  bool b_enable_topic_{false};
  // 点云数据
  std::shared_ptr<SensorData<sensor_msgs::LaserScan>> laser_data_ptr_{nullptr};
  // 点云数据显示所用
  std::shared_ptr<glk::PointCloudBuffer> pointcloud_buffer_ptr_{nullptr};
  // 当前pointcloud_buffer中数据的时间ns数
  uint32_t time_ns_{0};

 private:
  /// 绘制话题ui
  void draw_ui_topic_name();
  /// 检查当前设备在线状态
  void check_online_status();
};

}  // namespace dev
