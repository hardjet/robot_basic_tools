#pragma once

#include <memory>
#include <mutex>

#include <sensor_msgs/LaserScan.h>
#include "calib_base.hpp"

namespace cv_bridge {
class CvImage;
}

namespace cv {
class Mat;
}

namespace dev {
class AprilBoard;
class Camera;
class Laser;
class ImageShow;
}  // namespace dev

namespace calibration {

class Task;

class CamLaserCalib : public BaseCalib {
 public:
  CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                std::shared_ptr<dev::AprilBoard>& april_board_ptr_);

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;

 private:
  /// 更新数据
  void update_data();
  /// 检测图片是否有足够的apriltags
  bool get_cam_pose();
  /// 标定流程
  void calibration();

 private:
  // 是否显示图像
  bool is_show_image_{false};
  // 是否有新图像数据
  bool is_new_image_{false};
  // 是否有新激光数据
  bool is_new_laser_{false};
  // 资源锁
  std::mutex mtx_;
  // 标定板对象
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 当前选中的相机对象
  std::shared_ptr<dev::Camera> cam_ptr_{nullptr};
  // 当前选中的相机对象
  std::shared_ptr<dev::Laser> laser_ptr_{nullptr};
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> im_show_ptr_{nullptr};
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 当前图像对象
  boost::shared_ptr<const cv_bridge::CvImage> image_ptr_{nullptr};
  // 显示使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_image_ptr_{nullptr};
  // 激光数据
  boost::shared_ptr<const sensor_msgs::LaserScan> laser_data_ptr_{nullptr};

};
}  // namespace calibration
