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

  void free() override { free_model(); }

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;
  /// 获取当前图像数据
  boost::shared_ptr<cv_bridge::CvImage const> data();
  /// 获取当前相机对象
  boost::shared_ptr<camera_model::Camera> cam() { return inst_ptr_; };

 private:
  // 当前类型
  int current_camera_type_{2};
  // 是否显示图像
  bool b_show_image_{false};
  // ros topic 使能
  bool b_enable_topic_[2]{false, false};
  // 相机对象
  boost::shared_ptr<camera_model::Camera> inst_ptr_{nullptr};
  // 相机内参
  std::vector<double> inst_params_;
  // 获取图像数据
  std::shared_ptr<SensorData<sensor_msgs::Image>> image_data_ptr_;
  // 获取深度点云数据
  std::shared_ptr<SensorData<sensor_msgs::PointCloud2>> points_data_ptr_;
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> im_show_ptr_{nullptr};
  // 保存转码后的图像数据
  boost::shared_ptr<cv_bridge::CvImage const> image_cv_ptr_{nullptr};

 private:
  /// 创建指定类型的相机
  void creat_instance(int current_camera_type);
  /// 绘制参数ui
  void draw_ui_params();
  /// 绘制话题ui
  void draw_ui_topic_name();
  /// 检查当前设备在线状态
  void check_online_status();
  /// 需要将图像消息转为cv::Mat
  bool cv_convert(boost::shared_ptr<const sensor_msgs::Image>& msg);
};

}  // namespace dev
