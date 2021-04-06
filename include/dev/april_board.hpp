#pragma once

#include <boost/shared_ptr.hpp>

namespace aslam {
namespace cameras {
class GridCalibrationTargetAprilgrid;
}
}  // namespace aslam

namespace dev {

/**
 * @brief 标定板对象
 *
 */
class AprilBoard {
 public:
  explicit AprilBoard(std::string& data_path);

  /**
   * @brief 画ui
   */
  void draw_ui();

  /**
   * 画3d显示部分
   * @param shader
   */
  void draw_gl(glk::GLSLShader& shader);

  /**
   * @brief 打开显示ui开关
   */
  void show();

  /**
   * 获取sensor位姿
   * @param pose
   */
  const Eigen::Matrix4f& get_pose() const { return T_; }

  /**
   * 设置sensor位姿
   * @param new_pose
   */
  void set_pose(const Eigen::Matrix4f& new_pose) { T_ = new_pose; }

 public:
  // 标定板对象
  boost::shared_ptr<aslam::cameras::GridCalibrationTargetAprilgrid> board;

 private:
  // 图片texture id
  unsigned int texture_id_{0};
  // 图像宽度
  int img_width_{0};
  // 图像高度
  int img_height_{0};
  // 显示ui窗口
  bool is_show_window_{false};
  // tagSize
  double tag_size_{0.03};
  // tagSpacing
  double tag_spacing_{0.3};
  // 板子长度
  double board_lenght_{};
  // 设备在世界坐标系下的位姿
  Eigen::Matrix4f T_{};
};

}  // namespace dev
