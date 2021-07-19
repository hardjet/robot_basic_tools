#pragma once

#include <memory>
#include <mutex>
#include <array>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/types.hpp"

#include "calib_base.hpp"

namespace cv_bridge {
class CvImage;
}

namespace glk {
class SimpleLines;
}

namespace dev {
class AprilBoard;
class Camera;
class ImageShow;
}  // namespace dev

namespace calibration {

class Task;

class TwoCamerasCalib : public BaseCalib {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TwoCamerasCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                  std::shared_ptr<dev::AprilBoard>& april_board_ptr);

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;
  /// 得到nodehandle
  void pass_nh(const ros::NodeHandle& nh);
  /// 得到所有的frames
  void pass_major_frames(const std::vector<std::string>& major_frames);

 private:
  /// 更新数据
  void update();

  /// 更新相关位姿
  void update_related_pose();

  /// 标定流程
  void calibration();

  /// 获取一对儿有效的相机位姿
  bool get_valid_pose();

  /// 更新显示相关的内容
  // void update_3d_show();

  /// 在3d中显示标定数据
  void draw_calib_data(glk::GLSLShader& shader);

  /// 保证保存的帧角度不同
  void check_and_save();

  /// 计算
  bool calc();

  /// 清除标定数据不需要计算标志
  void clear_calib_data_flag();

  /// 设定相机2到相机1的变换矩阵
  void draw_ui_transform();

  /// 设置标定参数
  void draw_calib_params();

  /// 更新transform的值
  void update_ui_transform();

  /// 从文件加载标定数据
  bool load_calib_data(const std::string& file_path);

  /// 保存标定数据到文件
  bool save_calib_data(const std::string& file_path);

  /// 保存外参到文件
  bool save_extrinsics(const std::string& file_path);

 private:
  // 相机数据结构
  struct CameraInstType {
    int id;
    // 是否有新图像数据
    bool is_new_data{false};
    // 相机 frame id
    std::string frame_id;
    // 相机设备对象
    std::shared_ptr<dev::Camera> camera_dev_ptr{nullptr};
    // 相机显示设备对象
    std::shared_ptr<dev::ImageShow> img_show_dev_ptr{nullptr};
    // 接收到的图像数据
    boost::shared_ptr<const cv_bridge::CvImage> cv_image_data_ptr{nullptr};
    // 显示图像使用
    boost::shared_ptr<const cv_bridge::CvImage> show_cv_image_ptr{nullptr};

    /// 更新显示控件
    void update();
  };

  // 标定数据
  struct CalibData {
    // 时间戳
    std::array<double, 2> timestamp{};
    // 检测到的角点空间坐标
    std::array<std::vector<cv::Point2f>, 2> image_points;
    // 是否需要计算
    bool b_need_calc{true};
    // 图像点投射在标定板上的点
    std::array<std::vector<Eigen::Vector3d>, 2> pts_on_board;
    // 检测到的角点图像坐标
    std::array<std::vector<cv::Point3f>, 2> object_points;
    // 相机坐标系到aprilboard坐标系的变换矩阵
    std::array<Eigen::Quaterniond, 2> q_ac;
    std::array<Eigen::Vector3d, 2> t_ac;
  };

  // 是否显示图像
  bool b_show_image_{false};
  // 是否显示就绪的标定数据
  bool b_show_calib_data_{false};
  // 是否需要更新显示的标定数据
  bool b_need_to_update_cd_{false};
  // 控件使用，保存相机2到相机1的变换矩阵的值 [tx, ty, tz, roll, pitch, yaw]
  std::array<float, 6> transform_12_{};
  // 相机2到相机1的变换矩阵
  Eigen::Matrix4f T_12_;
  // 变换矩阵是否有效，只有正常标定计算出的结果才算有效
  bool is_transform_valid_{false};
  // 当前选中的数据
  uint32_t selected_calib_data_id_{1};

  // 资源锁
  std::mutex mtx_;
  // 标定板对象
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 2个相机实例
  std::array<CameraInstType, 2> camera_insts_;
  // 当前计算出的标定数据
  CalibData calib_data_;
  // 候选标定数据
  std::vector<CalibData> calib_data_vec_;
  // 有效的标定数据
  std::vector<CalibData> calib_valid_data_vec_;
  // 采集数据时抖动控制 距离
  double jitter_dist_{0.005};
  // 采集数据时抖动控制 角度
  double jitter_angle_{0.2};
  // 不同数据角度间隔 deg
  double between_angle_{1.0};

  ros::NodeHandle ros_nh_;
//  ros::Subscriber tf_static_sub_;

  std::set<std::string> major_frames_set_;
  std::vector<std::string> major_frames_list_;
//  std::shared_ptr<tf2_ros::Buffer> tfBuffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(30));
};
}  // namespace calibration
