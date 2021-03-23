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
  void update();

  /// 标定流程
  void calibration();

  /// 获取一对儿有效的激光数据
  bool get_valid_lines();

  /// 更新显示相关的内容
  void update_show();

  /// 在3d中显示标定数据
  void draw_calib_data(glk::GLSLShader& shader);

  /// 保证保存的帧角度不同
  void check_and_save();

  /// 计算
  bool calc();

  /// 从文件加载标定数据
  bool load_calib_data(const std::string& file_path);

  /// 保存标定数据到文件
  bool save_calib_data(const std::string& file_path);

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
    std::array<Eigen::Vector3d, 2> lines_params;
    // 直线y方向最大和最小值 [[min, max], [min, max]]
    std::array<Eigen::Vector2d, 2> lines_min_max;
    // 拟合出的两条直线
    std::shared_ptr<glk::SimpleLines> laser_lines_drawable_ptr{nullptr};
  };

  // 标定数据
  struct CalibData {
    // 时间戳
    double timestamp{0.};
    // 取第一条直线的角度
    double angle{0.};
    // 直线参数数据 [[l_1_a,l_1_b], [l_2_a,l_2_b]]
    std::array<std::array<Eigen::Vector3d, 2>, 2> lines_params;
    // 直线点数据 [[[l_1_a_s, l_1_a_e],[l_1_b_s, l_1_b_e]], [[l_2_a_s, l_2_a_e],[l_2_b_s, l_2_b_e]]]
    std::array<std::array<std::array<Eigen::Vector3f, 2>, 2>, 2> lines_pts;
    // 直线上的中点坐标 [[c_1_a,c_1_b], [c_2_a,c_2_b]]
    std::array<std::array<Eigen::Vector3d, 2>, 2> mid_pt_on_lines;
  };

  // 标定数据显示
  struct CalibDataShow{
    // 直线上的中点坐标
    std::array<Eigen::Vector3d, 2> mid_pt_on_lines;
    // 拟合出的两条直线
    std::shared_ptr<glk::SimpleLines> laser_lines_drawable_ptr{nullptr};
  };

  // 是否显示图像
  bool b_show_image_{false};
  // 是否显示就绪的标定数据
  bool b_show_calib_data_{false};
  // 是否需要更新显示的标定数据
  bool b_need_to_update_cd_{false};
  // 当前选中的数据
  uint32_t selected_calib_data_id_{1};
  // 标定数据显示
  std::array<CalibDataShow, 2> calib_show_data_;
  // 资源锁
  std::mutex mtx_;
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 2个激光实例
  std::array<LaserInstType, 2> laser_insts_;
  // 当前计算出的标定数据
  CalibData calib_data_;
  // 候选标定数据
  std::vector<CalibData> calib_data_vec_;
  // 有效的标定数据
  std::vector<CalibData> calib_valid_data_vec_;
};
}  // namespace calibration
