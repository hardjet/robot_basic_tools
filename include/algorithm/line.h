
#include <vector>
#include <memory>
#include <Eigen/Core>

namespace cv {
class Mat;
}

namespace algorithm {
class Line {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Line(const sensor_msgs::LaserScan& scan, double angle_range, double max_range);

  /// 找直线点
  bool find_line(std::vector<Eigen::Vector3d>& best_line_pts, cv::Mat& img);

  /// 使用ransac的方法找直线
  bool find_line_ransac(std::vector<Eigen::Vector3d>& best_line_pts, cv::Mat& img);

 private:
  /// 将LaserScan转换为点云
  void scan2points(const sensor_msgs::LaserScan& scan_in);

 private:
  // 图像显示的宽度
  const int img_w_ = 320;
  // 图像显示焦距
  const double img_focal_ = 450;
  // 拍照高度
  const double img_z_ = 4.0;
  // 直线搜索范围 单位m
  double max_range_{0.};
  // 角度范围[-angle_range_, +angle_range_] 单位deg
  double angle_range_{0.};
  // laserscan转换为点云，在有效区域
  std::vector<Eigen::Vector3d> points_;
  // laserscan转换为点云，有效区域外
  std::vector<Eigen::Vector3d> outlier_points_;
  // 最小角度
  double angle_min_{0.};
  // 最大角度
  double angle_max_{0.};
  // 角增量
  double angle_inc_{0.};

  // 以图像的方式显示
  std::shared_ptr<cv::Mat> img_ptr_;
};
}  // namespace algorithm