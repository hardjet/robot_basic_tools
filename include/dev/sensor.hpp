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

// sensor unique id
static uint32_t sensors_unique_id = 0;

enum SENSOR_TYPE { UNDEF = 0, CAMERA = 1, LASER, LIDAR, IMU };

/**
 * @brief Sensor object 抽象基类
 *
 */
class Sensor {
 public:
  using Ptr = std::shared_ptr<Sensor>;

  /**
  *
  * @param name 对象名称
  * @param ros_nh ros
  * @param type
  * @param type_str
  */
  Sensor(const std::string& name, ros::NodeHandle& ros_nh, SENSOR_TYPE type = SENSOR_TYPE::UNDEF,
         std::string type_str = "undef")
      : sensor_name(name), nh_(ros_nh), sensor_id(sensors_unique_id++), sensor_type(type), sensor_type_str(std::move(type_str)) {}

  /**
   * @brief 虚析构函数
   *
   */
  virtual ~Sensor() = default;

  /**
   * @brief 修改传感器名称
   * @param name 将传感器名称设置成
   */
  void change_sensor_name(const std::string& name) { sensor_name = name; }

  /**
  * @brief 获取当前在线状态
  * @return 在线状态
  */
  bool is_sensor_online() const { return is_online_; }

  /**
  * @brief 在ui上画当前状态
  *
  */
  void draw_status();

  /**
  * @brief 打开显示ui开关
  */
  void show();

  /**
  * @brief 将当前对象标记为删除状态
  */
  void marked_to_be_deleted() { is_to_be_deleted_ = true; }

  /**
  * @brief 获取当前对象删除标记
  * @return 当前状态是否需要被删除
  */
  bool is_to_be_deleted() const { return is_to_be_deleted_; }

  /**
  * @brief opengl渲染
  * @param shader
  */
  virtual void draw_gl(glk::GLSLShader& shader) = 0;

  /**
  * @brief imgui绘图
  */
  virtual void draw_ui() = 0;

 public:
  // 设备名称
  std::string sensor_name;
  // 设备唯一id
  const uint32_t sensor_id;
  // 设备类型
  const SENSOR_TYPE sensor_type;
  // 设备名称
  const std::string sensor_type_str;

 protected:
  // 设备在线颜色 绿色
  const std::vector<double> COLOR_ONLINE{173.0 / 255, 1., 47.0 / 255};
  // 设备离线颜色 红色
  const std::vector<double> COLOR_OFFLINE{204.0 / 255, 51.0 / 255, 51.0 / 255};

  // ros 句柄
  ros::NodeHandle& nh_;
  // ros话题列表
  std::vector<std::string> topic_list_;
  // 是否需要被删除
  bool is_to_be_deleted_{false};
  // 在线状态
  bool is_online_{false};
  // 是否显示ui窗口
  bool is_show_window_{false};
  // 3d模型
  // ...
};

}  // namespace dev
