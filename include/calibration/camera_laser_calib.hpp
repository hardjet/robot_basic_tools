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

namespace dev {
class AprilBoard;
class Camera;
class Laser;
class ImageShow;
}  // namespace dev

namespace glk {
class SimpleLines;
class PointCloudBuffer;
}  // namespace glk

namespace calibration {

class Task;

class CamLaserCalib : public BaseCalib {
 public:
  CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                std::shared_ptr<dev::AprilBoard>& april_board_ptr);

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;

  void change_current_state(std::shared_ptr<CalibrationState> new_state) override;

  void change_next_state(std::shared_ptr<CalibrationState> new_state) override;

  bool instrument_available() override;

  int pose_valid() override;

  void check_steady() override;

  int do_calib() override;

 private:
  /// 更新数据
  void update_data();
  /// 更新3d显示相关的内容
  void update_3d_show();
  /// 检测图片是否有足够的apriltags
  bool get_pose_and_points();
  /// 在3d中显示标定数据
  void draw_calib_data(glk::GLSLShader& shader);
  /// 设置标定参数
  void draw_calib_params();
  /// 标定流程
  void calibration();
  void new_calibration();
  /// 保证保存的帧角度不同
  void check_and_save();
  /// 更新相关的位姿
  void update_related_pose();
  /// 更新transform的值
  void update_ui_transform();
  /// 设定相机到激光的变换矩阵
  void draw_ui_transform();
  /// 计算
  bool calc();
  /// 从文件加载标定数据
  bool load_calib_data(const std::string& file_path);
  /// 保存标定数据到文件
  bool save_calib_data(const std::string& file_path);

 private:
  // 标定数据
  struct CalibData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    // 相机坐标系到世界坐标系的变换(这里的世界坐标系实际上是标定板的坐标系)
    Eigen::Quaterniond q_ac;
    Eigen::Vector3d t_ac;
    // 检测到的激光点数据
    std::vector<Eigen::Vector3d> line_pts;
    // 直线上的点
    std::vector<Eigen::Vector3d> pts_on_line;
    // 检测到的直线参数 Ax+By+C=0
    Eigen::Vector3d line_params;
  };

  // 是否显示图像
  bool b_show_image_{false};
  // 是否显示就绪的标定数据
  bool b_show_calib_data_{false};
  // 是否需要更新显示的标定数据
  bool b_need_to_update_cd_{false};
  // 是否有新图像数据
  bool is_new_image_{false};
  // 是否有新激光数据
  bool is_new_laser_{false};
  // 有效激光范围m
  double max_range_{1.0};
  // 激光角度范围 deg
  double angle_range_{30.0};
  // dist_thd 点到直线的距离门限m
  double dist_thd_{0.03};
  // 直线最少包含的点数
  uint32_t min_num_of_pts_{50};
  // 不同数据角度间隔 deg
  double between_angle_{3.0};
  // 当前选中的数据
  uint32_t selected_calib_data_id_{1};
  // 控件使用，保存相机到激光变换矩阵相关的值 [tx, ty, tz, roll, pitch, yaw]
  std::array<float, 6> transform_lc_{};
  // 激光2到激光1的变换矩阵
  Eigen::Matrix4f T_lc_;

  // 资源锁
  std::mutex mtx_;
  // 标定板对象
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 当前选中的相机对象
  std::shared_ptr<dev::Camera> cam_dev_ptr_{nullptr};
  // 当前选中的激光对象
  std::shared_ptr<dev::Laser> laser_dev_ptr_{nullptr};
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> image_imshow_ptr_{nullptr};
  // 激光显示对象
  std::shared_ptr<dev::ImageShow> laser_imshow_ptr_{nullptr};
  // 拟合出的直线，3d显示使用
  std::shared_ptr<glk::SimpleLines> laser_line_3d_ptr{nullptr};
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 当前图像对象
  boost::shared_ptr<const cv_bridge::CvImage> cv_image_ptr_{nullptr};
  // 显示相机使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_cam_cv_img_ptr_{nullptr};
  // 显示激光使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_laser_cv_img_ptr_{nullptr};
  // 激光原始数据
  boost::shared_ptr<const sensor_msgs::LaserScan> laser_raw_data_ptr_{nullptr};
  // 标定数据中点云数据显示所用
  std::shared_ptr<glk::PointCloudBuffer> calib_pointcloud_ptr_{nullptr};
  // 当前计算出的标定数据
  CalibData calib_data_;
  // 候选标定数据
  std::vector<CalibData> calib_data_vec_;
  // 有效的标定数据
  std::vector<CalibData> calib_valid_data_vec_;
};
}  // namespace calibration
