#pragma once

#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <Eigen/Core>

#include "glk/drawble.hpp"
#include "glk/text_renderer.hpp"
#include "guik/gl_canvas.hpp"

//  前向声明
namespace glk {
class GLSLShader;
}

namespace ros {
class NodeHandle;
}

namespace dev {

extern std::string config_default_path;
extern std::string data_default_path;

// sensor type
enum SENSOR_TYPE { UNDEF = 0, CAMERA, LASER, LIDAR, IMU };
extern const std::string dev_type_str[];

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
  Sensor(std::string name, ros::NodeHandle& ros_nh, SENSOR_TYPE type = SENSOR_TYPE::UNDEF)
      : sensor_name(std::move(name)),
        sensor_id(sensors_unique_id++),
        sensor_type(type),
        sensor_type_str(dev_type_str[type]),
        nh_(ros_nh) {
    T_ = Eigen::Matrix4f::Identity();
    //    T_(0,3) = 1.0;
  }

  /**
   * @brief 虚析构函数
   *
   */
  virtual ~Sensor();

  /*
   * @brief 释放资源
   */
  virtual void free() = 0;

  /**
   * @brief opengl渲染
   * @param shader
   */
  virtual void draw_gl(glk::GLSLShader& shader, const std::shared_ptr<guik::GLCanvas>& canvas_ptr) = 0;

  /**
   * @brief imgui绘图
   */
  virtual void draw_ui() = 0;

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
   * 在空间中绘制传感器的坐标系，表征传感器的位置
   * @param shader
   */
  void draw_gl_coordinate_system(glk::GLSLShader& shader, const std::shared_ptr<guik::GLCanvas>& canvas_ptr) const;

  /**
   * @brief 打开显示ui开关
   */
  void show();

  /**
   * @brief 将当前对象标记为删除状态
   */
  void marked_to_be_deleted() { b_to_be_deleted_ = true; }

  /**
   * @brief 获取当前对象删除标记
   * @return 当前状态是否需要被删除
   */
  bool b_to_be_deleted() const { return b_to_be_deleted_; }

  /**
   * 获取sensor位姿
   * @param pose
   */
  const Eigen::Matrix4f& get_sensor_pose() const { return T_; }

  /**
   * 设置sensor位姿
   * @param new_pose
   */
  void set_sensor_pose(const Eigen::Matrix4f& new_pose) { T_ = new_pose; }

 protected:
  /**
   * @brief 画数据颜色选择器
   */
  void draw_data_color_selector();

  /**
   * @brief 加载.ply模型文件
   * @return
   */
  bool load_model();

  /**
   * @brief 释放模型资源
   * @return
   */
  void free_model();

 public:
  // 设备名称
  std::string sensor_name;
  // 设备唯一id
  const uint32_t sensor_id;
  // 设备类型
  const SENSOR_TYPE sensor_type;
  // 设备类型名称
  const std::string sensor_type_str;
  // sensor unique id
  static uint32_t sensors_unique_id;

 protected:
  // 设备在线颜色 绿色
  const std::vector<float> COLOR_ONLINE{173.0 / 255, 1., 47.0 / 255};
  // 设备离线颜色 红色
  const std::vector<float> COLOR_OFFLINE{204.0 / 255, 51.0 / 255, 51.0 / 255};
  // 数据颜色
  std::vector<float> data_color_{1.0, 0., 0.};

  // ros 句柄
  ros::NodeHandle& nh_;
  // 传感器对应的ros话题
  std::vector<std::string> sensor_topic_list_;
  // 选择ros话题列表
  std::vector<std::string> ros_topic_selector_;
  // 是否需要被删除
  bool b_to_be_deleted_{false};
  // 在线状态
  bool is_online_{false};
  // 是否显示ui窗口
  bool b_show_window_{false};
  // 相机3d模型
  std::unique_ptr<glk::Drawable> ply_model_ptr_{nullptr};
  // 设备在世界坐标系下的位姿
  Eigen::Matrix4f T_{};
};

}  // namespace dev
