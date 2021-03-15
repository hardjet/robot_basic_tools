#pragma once

#include <memory>
#include <mutex>
#include <array>
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

namespace glk {
class SimpleLines;
}

namespace dev {
class Laser;
class ImageShow;
}  // namespace dev

namespace calibration {

class Task;

class TwoLasersCalib : public BaseCalib {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit TwoLasersCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr);

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;

 private:
  /// 更新数据
  void update_data();

  /// 标定流程
  void calibration();

  /// 获取一对儿有效的激光数据
  bool get_valid_points();

  /// 更新显示相关的内容
  void update_show();

 private:
  // 激光数据结构
  struct LaserInstType {
    int id;
    // 是否有新激光数据
    bool is_new_data{false};
    // 激光设备对象
    std::shared_ptr<dev::Laser> laser_dev_ptr{nullptr};
    // 激光显示设备对象
    std::shared_ptr<dev::ImageShow> img_show_dev_ptr{nullptr};
    // 激光数据
    boost::shared_ptr<const sensor_msgs::LaserScan> laser_data_ptr{nullptr};
    // 标定过程中锁定的激光数据
    boost::shared_ptr<const sensor_msgs::LaserScan> calib_data_ptr{nullptr};
    // 显示激光使用的image
    boost::shared_ptr<const cv_bridge::CvImage> img_ptr{nullptr};
    // 检测到的2条直线参数 Ax+By+C=0
    std::vector<Eigen::Vector3d> laser_lines_params;
    // 拟合出的两条直线
    std::shared_ptr<glk::SimpleLines> laser_lines_drawable_ptr{nullptr};
  };

  // 是否显示图像
  bool is_show_image_{false};
  // 资源锁
  std::mutex mtx_;
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 2个激光实例
  std::array<LaserInstType, 2> laser_insts_;
};
}  // namespace calibration
