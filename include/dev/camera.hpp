#pragma once

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "dev/sensor.hpp"

namespace camera_model {
class Camera;
}

namespace cv_bridge {
class CvImage;
}

namespace dev {

template <class M>
class SensorData;

class ImageShow;

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
  /// 获取当前图像数据
  boost::shared_ptr<sensor_msgs::Image const> data();

 private:
  // 当前类型
  int current_camera_type_{2};
  // 是否显示图像
  bool is_show_image_{false};

  // 相机对象
  boost::shared_ptr<camera_model::Camera> inst_ptr_{nullptr};
  // 相机内参
  std::vector<double> inst_params_;
  // ros topic 使能
  bool enable_topic_[2]{false, false};
  // 图像数据
  std::shared_ptr<SensorData<sensor_msgs::Image>> image_data_;
  // 深度点云数据
  std::shared_ptr<SensorData<sensor_msgs::PointCloud2>> points_data_;
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> im_show_ptr_{nullptr};

 private:
  /// 创建指定类型的相机
  void creat_instance(int current_camera_type);
  /// 绘制参数ui
  void draw_ui_parms();
  /// 绘制话题ui
  void draw_ui_topic_name();
  /// 检查当前设备在线状态
  void check_online_status();
};

}  // namespace dev
