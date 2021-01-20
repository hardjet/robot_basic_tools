#pragma once

#include <unordered_map>
#include <list>

#include "dev/sensor.hpp"

namespace dev {

/**
 * @brief SensorManager object
 *
 */
class SensorManager {
 public:
  using Ptr = std::shared_ptr<SensorManager>;

  SensorManager(ros::NodeHandle& ros_nh) : nh_(ros_nh){};

  ~SensorManager();

  // 添加传感器
  void add_sensor(dev::Sensor::Ptr& sensor);

  // ui
  void draw_ui();

 private:
  // 检查并清楚需要删除的传感器
  void check_and_clear_sensors();

 private:
  // ros 句柄
  ros::NodeHandle& nh_;
  // 便于删除操作
  std::unordered_map<dev::SENSOR_TYPE, std::list<dev::Sensor::Ptr>> sensors_map_;
};

}  // namespace dev
