#pragma once

#include <memory>
#include <mutex>
#include <array>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
// #include <Eigen/Geometry>

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

 private:
  /// 更新数据
  void update();

  /// 标定流程
  void calibration();

  /// 获取一对儿有效的相机位姿
  bool get_valid_pose();

  /// 更新显示相关的内容
  void update_3d_show();

  /// 在3d中显示标定数据
  void draw_calib_data(glk::GLSLShader& shader);

  /// 保证保存的帧角度不同
  void check_and_save();

  /// 计算
  bool calc();

  /// 设定相机2到相机1的变换矩阵
  void draw_ui_transform();

  /// 设置标定参数
  void draw_calib_params();

  /// 更新transform的值
  void update_ui_transform();

  /// 更新相机2位姿
  void update_camera2_pose();

  /// 从文件加载标定数据
  bool load_calib_data(const std::string& file_path);

  /// 保存标定数据到文件
  bool save_calib_data(const std::string& file_path);

 private:
  // 激光数据结构
  struct CameraInstType {
    int id;
    // 是否有新图像数据
    bool is_new_data{false};
    // 激光设备对象
    std::shared_ptr<dev::Camera> camera_dev_ptr{nullptr};
    // 激光显示设备对象
    std::shared_ptr<dev::ImageShow> img_show_dev_ptr{nullptr};
    // 接收到图像数据
    boost::shared_ptr<const cv_bridge::CvImage> cv_image_data_ptr{nullptr};
    // 显示图像使用
    boost::shared_ptr<const cv_bridge::CvImage> show_cv_image_ptr{nullptr};
  };

  // 标定数据
  struct CalibData {
    // 时间戳
    double timestamp{0.};
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
  // 当前选中的数据
  uint32_t selected_calib_data_id_{1};

  // 资源锁
  std::mutex mtx_;
  // 标定板对象
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 2个激光实例
  std::array<CameraInstType, 2> camera_insts_;
  // 当前计算出的标定数据
  CalibData calib_data_;
  // 候选标定数据
  std::vector<CalibData> calib_data_vec_;
  // 有效的标定数据
  std::vector<CalibData> calib_valid_data_vec_;
  // 不同数据角度间隔 deg
  double between_angle_{1.0};
};
}  // namespace calibration
