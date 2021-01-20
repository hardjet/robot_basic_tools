#pragma once

#include <string>
#include <vector>
#include <memory>

//  前向声明
namespace glk {
class GLSLShader;
}

namespace ros {
class NodeHandle;
}

namespace dev {

enum SENSOR_TYPE { UNDEF = 0, CAMERA = 1, LASER, LIDAR, IMU };

/**
 * @brief Sensor object 抽象基类
 *
 */
class Sensor {
 public:
  using Ptr = std::shared_ptr<Sensor>;

  Sensor(const std::string& name, ros::NodeHandle& ros_nh, SENSOR_TYPE type = SENSOR_TYPE::UNDEF,
         std::string type_str = "undef")
      : sensor_name(name), nh_(ros_nh), sensor_type(type), sensor_type_str(std::move(type_str)) {}

  // 虚析构函数
  virtual ~Sensor() = default;
  // 修改传感器名称
  void change_sensor_name(const std::string& name) { sensor_name = name; }
  // 获取当前在线状态
  bool is_sensor_online() const { return is_online_; }

  // opengl渲染
  virtual void draw_gl(glk::GLSLShader& shader) = 0;
  // imgui绘图
  virtual void draw_ul() = 0;

  // 画当前状态
  void draw_status();

  // 标记删除
  void marked_to_be_deleted() { is_to_be_deleted_ = true; }
  // 获取删除标记
  bool is_to_be_deleted() const {return is_to_be_deleted_;}

 public:
  // 设备名称
  std::string sensor_name;
  // 设备类型
  const SENSOR_TYPE sensor_type;
  // 设备名称
  const std::string sensor_type_str;

 protected:
  // ros 句柄
  ros::NodeHandle& nh_;
  // ros话题列表
  std::vector<std::string> topic_list_;
  // 是否需要被删除
  bool is_to_be_deleted_{false};
  // 在线状态
  bool is_online_{false};
  // 设备在线颜色 绿色
  const std::vector<double> COLOR_ONLINE{173.0 / 255, 1., 47.0 / 255};
  // 设备离线颜色 红色
  const std::vector<double> COLOR_OFFLINE{204.0 / 255, 51.0 / 255, 51.0 / 255};
  // 3d模型
  // ...
};

}  // namespace dev
