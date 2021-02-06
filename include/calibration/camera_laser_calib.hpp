#pragma once

#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
  // 标定数据
  struct CalibData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    // 相机坐标系到世界坐标系的变换
    Eigen::Quaterniond q_wc;
    Eigen::Vector3d t_wc;
    // 检测到的激光点数据
    std::vector<Eigen::Vector3d> line_pts;
    // 显示使用的image
    boost::shared_ptr<const cv_bridge::CvImage> cam_img_ptr_{nullptr};
    // 显示激光使用的image
    boost::shared_ptr<const cv_bridge::CvImage> laser_img_ptr_{nullptr};
  };

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
  bool get_pose_and_points();
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
  std::shared_ptr<dev::ImageShow> im_show_dev_ptr_{nullptr};
  // 激光显示对象
  std::shared_ptr<dev::ImageShow> laser_show_dev_ptr_{nullptr};
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 当前图像对象
  boost::shared_ptr<const cv_bridge::CvImage> image_ptr_{nullptr};
  // 显示使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_cam_img_ptr_{nullptr};
  // 显示激光使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_laser_img_ptr_{nullptr};
  // 激光数据
  boost::shared_ptr<const sensor_msgs::LaserScan> laser_data_ptr_{nullptr};
  // 当前计算出的标定数据
  std::shared_ptr<CalibData> calib_data_{nullptr};
  // 候选标定数据
  std::vector<CalibData> calib_data_vec_;
  // 有效的标定数据
  std::vector<CalibData> calib_valid_data_vec_;
};
}  // namespace calibration
