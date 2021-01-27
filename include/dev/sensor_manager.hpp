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

  explicit SensorManager(ros::NodeHandle& ros_nh) : nh_(ros_nh){};

  ~SensorManager();

  /**
   * @brief 添加传感器
   * @param sensor
   */
  void add_sensor(dev::Sensor::Ptr& sensor);

  /**
   * @brief 画ui
   */
  void draw_ui();

 private:
  /// 检查并清楚需要删除的传感器
  void check_and_clear_sensors();

  /// 调用所有传感器的draw_ui
  void call_sensors_draw_ui();

 public:
  // 使用unordered_map，list便于删除操作
  std::unordered_map<dev::SENSOR_TYPE, std::list<dev::Sensor::Ptr>> sensors_map;

 private:
  // ros 句柄
  ros::NodeHandle& nh_;

  // 设备个数
  int sensor_num_{0};
};

}  // namespace dev
