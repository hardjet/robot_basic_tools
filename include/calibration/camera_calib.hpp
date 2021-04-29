#pragma once

#include <memory>
#include <mutex>
#include <array>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/types.hpp"

#include "calib_base.hpp"
#include "camera_model/calib/CameraCalibration.h"
namespace cv_bridge {
class CvImage;
}

namespace glk {
class PointCloudBuffer;
}

namespace dev {
class AprilBoard;
class chessboard;
class Camera;
class ImageShow;
}  // namespace dev

namespace calibration {

class Task;

class CameraCalib : public BaseCalib {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
              std::shared_ptr<dev::AprilBoard>& april_board_ptr,
              std::shared_ptr<dev::chessboard>& chess_board_ptr);
  // opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  // im gui绘图
  void draw_ui() override;

 private:
  //图像旋转不连续检测
  bool get_pose_and_points();
  //进行矫正的基本配置
  void calibration();
  void update_data();
  /// 保证保存的帧角度不同
  void check_and_save();
  // opengl显示矫正数据
  void draw_calib_data(glk::GLSLShader& shader);
  void update_3d_show();
  //核心矫正算法
  bool calc();
  /// 设置标定参数
  void draw_calib_params();
  //显示出内参信息
  void draw_ui_params();
  //保存矫正过程中产生的数据
  bool save_calib_data(const std::string& file_path);
  //加载矫正过程中产生的数据
  bool load_calib_data(const std::string& file_path);

 private:
  // 是否使用标准棋盘
  bool USE_APRIL_BOARD{true};
  // AprilBoard标定板对象
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 标准棋盘格标定板对象
  std::shared_ptr<dev::chessboard> chess_board_ptr_;
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> image_imshow_ptr_{nullptr};
  // 标定数据中点云数据显示所用
  std::shared_ptr<glk::PointCloudBuffer> calib_pointcloud_ptr_{nullptr};
  // 当前选中的数据
  uint32_t selected_calib_data_id_{1};
  // 是否需要更新显示的标定数据
  bool b_need_to_update_cd_{false};
  //是否显示图像
  bool b_show_image_{false};
  // 是否有新图像数据
  bool is_new_image_{false};
  // 是否显示就绪的标定数据
  bool b_show_calib_data_{false};
  // 不同数据角度间隔 deg
  double between_angle_{3.0};
  //保存的相机配置yaml存储路径
  std::string save_yaml_path;
  // 相机内参
  std::vector<double> inst_params_;
  // 标定数据
  struct CalibData {
    double timestamp;
    // 相机坐标系到世界坐标系的变换(这里的世界坐标系实际上是标定板的坐标系)
    Eigen::Quaterniond q_ac;
    Eigen::Vector3d t_ac;
    // 是否需要计算
    bool b_need_calc{true};
    // 图像点投射在标定板上的点
    std::vector<Eigen::Vector3d> pts_on_board;
    // 检测到的角点空间坐标
    std::vector<cv::Point2f> imagePoints;
    // 检测到的角点图像坐标
    std::vector<cv::Point3f> objectPoints;
    //需要显示的图像
    cv::Mat pic_show;
  };
  // 资源锁
  std::mutex mtx_;
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  std::shared_ptr<dev::Camera> cam_dev_ptr_{nullptr};

  // 当前计算出的标定数据
  CalibData calib_data_;
  // 候选标定数据
  std::vector<CalibData> calib_data_vec_;
  // 有效的标定数据
  std::vector<CalibData> calib_valid_data_vec_;
  // 显示相机使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_cam_cv_img_ptr_{nullptr};
  // 当前图像对象
  boost::shared_ptr<const cv_bridge::CvImage> cv_image_ptr_{nullptr};
};
}  // namespace calibration
