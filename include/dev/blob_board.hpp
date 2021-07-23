#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <opencv2/calib3d.hpp>
namespace glk {
class GLSLShader;
class Drawable;
}  // namespace glk
namespace dev {
/**
 * @brief 棋盘格标定板对象
 *
 */
class blob_board{
 public:
  explicit blob_board(std::string& data_path);
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
  void show() { b_show_window_ = true; };
  /**
   * @brief 打开显示3d开关
   */
  void show_3d() { b_show_3d_ = true; };
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
  cv::Size get_board_size()const
  {
    return cv::Size{tag_rows_,tag_cols_};
  }
  double  get_tag_size_()const
  {
    return tag_size_;
  }
 private:
  /// 更新chessboard 3d边框显示相关
  void update_blob_board_edges();

 private:
  // 图片texture id
  unsigned int texture_id_{0};
  // 图像宽度
  int img_width_{0};
  // 图像高度
  int img_height_{0};
  // 显示ui窗口
  bool b_show_window_{false};
  // 是否显示3d内容
  bool b_show_3d_{false};
  // tagRows
  int tag_rows_{4};
  // tagCols
  int tag_cols_{8};
  // tagSize
  double tag_size_{0.052};
  // 板子长度
  double board_lenght_{};
  // 板子高度
  double board_height_{};
  // 显示角点 ---- 弃用，点没有深度，无法正常显示
  // std::shared_ptr<glk::PointCloudBuffer> points_3d_ptr_{nullptr};
  // chess board上的角点
  std::vector<Eigen::Vector3d> blob_board_points_;
  // april board的边
  std::shared_ptr<glk::Drawable> blob_edges_ptr_{nullptr};
  // 设备在世界坐标系下的位姿
  Eigen::Matrix4f T_{};
};

}  // namespace dev
