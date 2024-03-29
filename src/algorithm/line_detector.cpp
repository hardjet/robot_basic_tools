#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <utility>

#include "algorithm/util.h"
#include "algorithm/line_detector.h"
#include "algorithm/ransac/ransac_applications.h"

namespace algorithm {

struct LineSeg {
  int id_start;
  int id_end;
  double dist;
};

LineDetector::LineDetector(const sensor_msgs::LaserScan& scan, double angle_range, double max_range)
    : max_range_(max_range), angle_range_(DEG2RAD_RBT(angle_range)) {
  points_.resize(0);
  outlier_points_.resize(0);
  img_ptr_ = std::make_shared<cv::Mat>(img_w_, img_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  // 转换点
  scan2points(scan);
}

LineDetector::LineDetector(const sensor_msgs::LaserScan& scan, double angle_range, double max_range, int point_limit,
                           int inlier_limit, double thd_dist, double z, Eigen::Matrix<double, 3, 3> r,
                           Eigen::Vector3d t, int id)
    : max_range_(max_range),
      angle_range_(DEG2RAD_RBT(angle_range)),
      thd_points_(point_limit),
      thd_inliers_(inlier_limit),
      thd_dist_(thd_dist),
      ortho_z_(z),
      rotation_(std::move(r)),
      translation_(std::move(t)),
      id_(id) {
  points_.resize(0);
  outlier_points_.resize(0);
  img_ptr_ = std::make_shared<cv::Mat>(img_w_, img_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  ortho_ptr_ = std::make_shared<cv::Mat>(img_w_, img_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  pc_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  // 转换点
  scan2points(scan);
}

void LineDetector::scan2points(const sensor_msgs::LaserScan& scan_in) {
  size_t n_pts = scan_in.ranges.size();
  //  printf("----- ----- LineDetector::scan2points() ..... n_pts = %zu\n", n_pts);

  double angle = scan_in.angle_min;
  double x, y;
  for (size_t i = 0; i < n_pts; ++i) {
    x = scan_in.ranges[i] * cos(angle);
    y = scan_in.ranges[i] * sin(angle);
    Eigen::Vector3d current_point = Eigen::Vector3d{x, y, 0};
    Eigen::Vector3d current_point_projected = rotation_ * current_point + translation_;

    // 范围内的点
    if (angle > -angle_range_ && angle < angle_range_ && scan_in.ranges[i] < max_range_) {
      points_.emplace_back(x, y, 0);
      points_projected_.push_back(current_point_projected);
    } else {
      outlier_points_.emplace_back(x, y, 0);
    }

    if (id_ == 0) {
      if (!isnan(current_point.x()) && !isnan(current_point.y()) && !isnan(current_point.z())) {
        pcl::PointXYZ p{float(current_point.x()), float(current_point.y()), 0.};
        pc_->push_back(p);
      }
    } else {
      if (!isnan(current_point_projected.x()) && !isnan(current_point_projected.y()) &&
          !isnan(current_point_projected.z())) {
        pcl::PointXYZ p{float(current_point_projected.x()), float(current_point_projected.y()), 0.};
        pc_->push_back(p);
      }
    }

    // 角增量
    angle += scan_in.angle_increment;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LineDetector::get_pc() { return pc_; }

bool LineDetector::find_two_lines(std::array<Eigen::Vector3d, 2>& lines_params,
                                  std::array<Eigen::Vector2d, 2>& lines_min_max, cv::Mat& img, cv::Mat& ortho) const {
  img = img_ptr_->clone();
  if (id_ == 0) {
    ortho = ortho_ptr_->clone();
  }

  // 直线点集合
  std::vector<std::vector<Eigen::Vector3d>> two_lines_pts(2);

  // ransac 数据
  std::vector<double> xs;
  std::vector<double> ys;
  // 检测到的直线 <点数，直线方程系数Ax+By+C=0>
  std::vector<std::pair<size_t, Eigen::Vector3d>> detectedLines;
  // 时间统计
  // mrpt::system::CTicTac tictac;
  // 点序号
  int point_index = 0;

  // 先显示所有点
  // 范围内的点
  for (auto pt : points_) {
    int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
    // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);

    int col_base = (int)(pt.x() / ortho_z_ * img_focal_ + img_w_ / 2);
    int row_base = (int)(-pt.y() / ortho_z_ * img_focal_ + img_w_ / 2);
    int col_projected = (int)(points_projected_[point_index].x() / ortho_z_ * img_focal_ + img_w_ / 2);
    int row_projected = (int)(-points_projected_[point_index].y() / ortho_z_ * img_focal_ + img_w_ / 2);
    point_index++;

    // 添加数据 不能有0数据
    if (fabs(pt.x()) > 1e-3 && fabs(pt.y()) > 1e-3) {
      xs.push_back(pt.x());
      ys.push_back(pt.y());
      // std::cout << pt.x() << "," << pt.y() << std::endl;
    }

    // 边界判断
    if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;
    if (id_ == 0) {
      if (col_base > img_w_ - 1 || col_base < 0 || row_base > img_w_ - 1 || row_base < 0) continue;
    } else {
      if (col_projected > img_w_ - 1 || col_projected < 0 || row_projected > img_w_ - 1 || row_projected < 0) continue;
    }

    cv::Vec3b color_green(0, 255, 0);
    cv::Vec3b color_blue(200, 0, 0);
    img.at<cv::Vec3b>(row, col) = color_green;

    if (id_ == 0) {
      ortho.at<cv::Vec3b>(row_base, col_base) = color_green;
    } else {
      ortho.at<cv::Vec3b>(row_projected, col_projected) = color_blue;
    }
  }

  // ---- 画坐标
  // x轴向前，y轴向左
  int orig_col = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
  int orig_row = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);

  int x_axis_col = (int)(0.2 / img_z_ * img_focal_ + img_w_ / 2);
  int x_axis_row = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
  cv::line(img, cv::Point{orig_col, orig_row}, cv::Point{x_axis_col, x_axis_row}, cv::Scalar(255, 0, 0), 4);
  cv::line(ortho, cv::Point{orig_col, orig_row}, cv::Point{x_axis_col, x_axis_row}, cv::Scalar(255, 0, 0), 4);

  int y_axis_col = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
  int y_axis_row = (int)(-0.2 / img_z_ * img_focal_ + img_w_ / 2);

  cv::line(img, cv::Point{orig_col, orig_row}, cv::Point{y_axis_col, y_axis_row}, cv::Scalar(0, 255, 0), 4);
  cv::line(ortho, cv::Point{orig_col, orig_row}, cv::Point{y_axis_col, y_axis_row}, cv::Scalar(0, 255, 0), 4);

  //  std::cout << "valid pts num: " << xs.size() << std::endl;

  // 点数过少
  // printf("----- LineDetector::find_two_lines() ..... xs.size() = %zu\tys.size() = %zu\n", xs.size(), ys.size());
  if (xs.size() < thd_points_) {
    printf("not enough points for xs and ys, returning false\n");
    return false;
  }

  // for (int i = 0; i < xs.size();) {
  //   printf("[%d]:%f, %f\n", i, xs[i], ys[i]);
  //   i += 10;
  // }

  // 直线内点距离门限5cm，有效点数门限50个
  Eigen::Map<Eigen::VectorXd> x_e(xs.data(), xs.size());
  Eigen::Map<Eigen::VectorXd> y_e(ys.data(), ys.size());

  //  std::cout << "ransac_detect_2D_lines in --------------" << std::endl;
  //  double thd = 0.05;
  double thd = thd_dist_;
  algorithm::ransac::ransac_detect_2D_lines(x_e, y_e, detectedLines, thd, thd_inliers_);
  //  std::cout << "-------------- ransac_detect_2D_lines out " << std::endl;

  // 检测到两条直线认为有效
  if (detectedLines.size() != 2) {
    //    std::cout << "detected lines num != 2. " << detectedLines.size() << std::endl;
    return false;
  }

  // 初始化
  lines_min_max[0](0) = 1000.;
  lines_min_max[0](1) = -1000.;
  lines_min_max[1](0) = 1000.;
  lines_min_max[1](1) = -1000.;

  // 计算点到直线的距离
  auto distance = [](const Eigen::Vector3d& line, double x, double y) {
    return std::abs(line[0] * x + line[1] * y + line[2]) / sqrt(line[0] * line[0] + line[1] * line[1]);
  };

  // 提取出直线对应的点 todo ransac算法里已经有，后面直接把数据传出来
  double dist;
  for (size_t i = 0; i < xs.size(); i++) {
    // mrpt::math::TPoint2D pt{xs(i, 0), ys(i, 0)};
    // dist = detectedLines[0].second.distance(pt);
    dist = distance(detectedLines[0].second, xs.at(i), ys.at(i));
    if (dist < thd) {
      two_lines_pts[0].emplace_back(Eigen::Vector3d{xs.at(i), ys.at(i), 0.});

      // 更新最值
      if (lines_min_max[0](1) < ys.at(i)) {
        lines_min_max[0](1) = ys.at(i);
      }

      if (lines_min_max[0](0) > ys.at(i)) {
        lines_min_max[0](0) = ys.at(i);
      }

    } else {
      dist = distance(detectedLines[1].second, xs.at(i), ys.at(i));
      if (dist < thd) {
        two_lines_pts[1].emplace_back(Eigen::Vector3d{xs.at(i), ys.at(i), 0.});

        // 更新最值
        if (lines_min_max[1](1) < ys.at(i)) {
          lines_min_max[1](1) = ys.at(i);
        }

        if (lines_min_max[1](0) > ys.at(i)) {
          lines_min_max[1](0) = ys.at(i);
        }
      }
    }
  }

  // 打印最大最小值
  // printf("0 min: %f, max:%f\n", lines_min_max[0](0), lines_min_max[0](1));
  // printf("1 min: %f, max:%f\n", lines_min_max[1](0), lines_min_max[1](1));

  // 打印直线提取相关信息
  // std::cout << " ransac_detect_2D_lines using: " << tictac.Tac() * 1000.0 << " ms" << std::endl;
  for (int i = 0; i < 2; i++) {
    // printf("line %d: ransac pts[%lu]-[%zu], line param:[%f, %f, %f]\n", i, detectedLines[i].first,
    //        two_lines_pts[i].size(), detectedLines[i].second.coefs[0], detectedLines[i].second.coefs[1],
    //        detectedLines[i].second.coefs[2]);

    // 在2d图上画直线
    double y1 = -2.0;
    double x1 = (-detectedLines[i].second[1] * y1 - detectedLines[i].second[2]) / detectedLines[i].second[0];
    int col_1 = (int)(x1 / img_z_ * img_focal_ + img_w_ / 2);
    // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    int row_1 = (int)(-y1 / img_z_ * img_focal_ + img_w_ / 2);

    double y2 = 2.0;
    double x2 = (-detectedLines[i].second[1] * y2 - detectedLines[i].second[2]) / detectedLines[i].second[0];
    int col_2 = (int)(x2 / img_z_ * img_focal_ + img_w_ / 2);
    // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    int row_2 = (int)(-y2 / img_z_ * img_focal_ + img_w_ / 2);

    // printf("ransan [%f, %f]{%d, %d} - [%f, %f]{%d, %d}\n", y1, x1, row_1, col_1, y2, x2, row_2, col_2);

    // 画直线
    cv::line(img, cv::Point{col_1, row_1}, cv::Point{col_2, row_2}, cv::Scalar(150, 0, 0), 1);

    // 最小二乘拟合
    // 直线模型 Ax + By + 1 = 0
    Eigen::Vector2d fitting_line{detectedLines[i].second[0] / detectedLines[i].second[2],
                                 detectedLines[i].second[1] / detectedLines[i].second[2]};
    if (!line_fitting_ceres(two_lines_pts[i], fitting_line)) {
      std::cout << "fitting line failed!" << detectedLines.size() << std::endl;
      return false;
    }

    // printf("fitting line(Ax+By+1=0):[%f, %f]\n", fitting_line[0], fitting_line[1]);

    x1 = (-fitting_line[1] * y1 - 1.) / fitting_line[0];
    col_1 = (int)(x1 / img_z_ * img_focal_ + img_w_ / 2);
    row_1 = (int)(-y1 / img_z_ * img_focal_ + img_w_ / 2);

    x2 = (-fitting_line[1] * y2 - 1.) / fitting_line[0];
    col_2 = (int)(x2 / img_z_ * img_focal_ + img_w_ / 2);
    row_2 = (int)(-y2 / img_z_ * img_focal_ + img_w_ / 2);

    // printf("fitting [%f, %f]{%d, %d} - [%f, %f]{%d, %d}\n", y1, x1, row_1, col_1, y2, x2, row_2, col_2);
    // 画直线
    cv::line(img, cv::Point{col_1, row_1}, cv::Point{col_2, row_2}, cv::Scalar(0, 0, 150), 1);

    // 保存直线方程系数
    lines_params[i] = Eigen::Vector3d{fitting_line[0], fitting_line[1], 1.};
  }

  return true;
}

bool LineDetector::find_line_ransac(std::vector<Eigen::Vector3d>& line_pts, Eigen::Vector3d& line_params, cv::Mat& img,
                                    double dist_thd, uint32_t min_num_of_pts) const {
  img = img_ptr_->clone();
  line_pts.clear();

  // ransac 数据
  std::vector<double> xs;
  std::vector<double> ys;
  // 检测到的直线 <点数，直线方程系数Ax+By+C=0>
  std::vector<std::pair<size_t, Eigen::Vector3d>> detectedLines;

  // 先显示所有点
  // 范围内的点
  for (auto pt : points_) {
    int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
    // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);

    // 添加数据 不能有0数据
    if (fabs(pt.x()) > 1e-3 && fabs(pt.y()) > 1e-3) {
      xs.push_back(pt.x());
      ys.push_back(pt.y());
      // std::cout << pt.x() << "," << pt.y() << std::endl;
    }

    if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;

    cv::Vec3b color_value(0, 255, 0);
    img.at<cv::Vec3b>(row, col) = color_value;
  }

  // ---- 画坐标
  // x轴向前，y轴向左
  int orig_col = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
  int orig_row = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);

  int x_axis_col = (int)(0.2 / img_z_ * img_focal_ + img_w_ / 2);
  int x_axis_row = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
  cv::line(img, cv::Point{orig_col, orig_row}, cv::Point{x_axis_col, x_axis_row}, cv::Scalar(255, 0, 0), 4);

  int y_axis_col = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
  int y_axis_row = (int)(-0.2 / img_z_ * img_focal_ + img_w_ / 2);

  cv::line(img, cv::Point{orig_col, orig_row}, cv::Point{y_axis_col, y_axis_row}, cv::Scalar(0, 255, 0), 4);

  // std::cout << "valid pts num: " << xs.size() << std::endl;

  // 点数过少
  if (xs.size() < min_num_of_pts) {
    return false;
  }

  // for (int i = 0; i < xs.size();) {
  //   printf("[%d]:%f, %f\n", i, xs[i], ys[i]);
  //   i += 10;
  // }

  // 直线内点距离门限5cm，有效点数门限50个
  Eigen::Map<Eigen::VectorXd> x_e(xs.data(), xs.size());
  Eigen::Map<Eigen::VectorXd> y_e(ys.data(), ys.size());

  // std::cout << "ransac_detect_2D_lines in --------------" << std::endl;

  algorithm::ransac::ransac_detect_2D_lines(x_e, y_e, detectedLines, dist_thd, min_num_of_pts);
  // std::cout << "-------------- ransac_detect_2D_lines out " << std::endl;

  // 检测到两条直线认为有效
  if (detectedLines.size() != 1) {
    std::cout << "detected lines num != 1. " << detectedLines.size() << std::endl;
    return false;
  }

  // 计算点到直线的距离
  auto distance = [](const Eigen::Vector3d& line, double x, double y) {
    return std::abs(line[0] * x + line[1] * y + line[2]) / sqrt(line[0] * line[0] + line[1] * line[1]);
  };

  // 提取出直线对应的点 todo ransac算法里已经有，后面直接把数据传出来
  double dist;
  for (size_t i = 0; i < xs.size(); i++) {
    // mrpt::math::TPoint2D pt{xs(i, 0), ys(i, 0)};
    // dist = detectedLines[0].second.distance(pt);
    dist = distance(detectedLines[0].second, xs.at(i), ys.at(i));
    if (dist < dist_thd) {
      line_pts.emplace_back(Eigen::Vector3d{xs.at(i), ys.at(i), 0.});
    }
  }

  // ---------------- 最小二乘拟合
  // 直线模型 Ax + By + 1 = 0
  Eigen::Vector2d fitting_line{detectedLines[0].second[0] / detectedLines[0].second[2],
                               detectedLines[0].second[1] / detectedLines[0].second[2]};
  if (!line_fitting_ceres(line_pts, fitting_line)) {
    std::cout << "fitting line failed!" << detectedLines.size() << std::endl;
    return false;
  }

  // 组成直线的点
  for (int j = 0; j < line_pts.size(); ++j) {
    Eigen::Vector3d pt = line_pts.at(j);
    int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
    int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果

    if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;

    cv::Vec3b color_value(238, 105, 17);
    img.at<cv::Vec3b>(row, col) = color_value;
  }

  // 在2d图上画直线
  double y1 = -2.0;
  double x1 = (-detectedLines[0].second[1] * y1 - detectedLines[0].second[2]) / detectedLines[0].second[0];
  int col_1 = (int)(x1 / img_z_ * img_focal_ + img_w_ / 2);
  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
  int row_1 = (int)(-y1 / img_z_ * img_focal_ + img_w_ / 2);

  double y2 = 2.0;
  double x2 = (-detectedLines[0].second[1] * y2 - detectedLines[0].second[2]) / detectedLines[0].second[0];
  int col_2 = (int)(x2 / img_z_ * img_focal_ + img_w_ / 2);
  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
  int row_2 = (int)(-y2 / img_z_ * img_focal_ + img_w_ / 2);

  // printf("ransan line: %fx + (%f)y + (%f) = 0\n", detectedLines[0].second[0], detectedLines[0].second[1],
  //        detectedLines[0].second[2]);
  // printf("ransan [%f, %f]{%d, %d} - [%f, %f]{%d, %d}\n", y1, x1, row_1, col_1, y2, x2, row_2, col_2);

  // 画直线
  cv::line(img, cv::Point{col_1, row_1}, cv::Point{col_2, row_2}, cv::Scalar(150, 0, 0), 1);

  // printf("fitting line: %fx + (%f)y + 1 = 0\n", fitting_line[0], fitting_line[1]);

  x1 = (-fitting_line[1] * y1 - 1.) / fitting_line[0];
  col_1 = (int)(x1 / img_z_ * img_focal_ + img_w_ / 2);
  row_1 = (int)(-y1 / img_z_ * img_focal_ + img_w_ / 2);

  x2 = (-fitting_line[1] * y2 - 1.) / fitting_line[0];
  col_2 = (int)(x2 / img_z_ * img_focal_ + img_w_ / 2);
  row_2 = (int)(-y2 / img_z_ * img_focal_ + img_w_ / 2);

  // printf("fitting [%f, %f]{%d, %d} - [%f, %f]{%d, %d}\n", y1, x1, row_1, col_1, y2, x2, row_2, col_2);
  // 画直线
  cv::line(img, cv::Point{col_1, row_1}, cv::Point{col_2, row_2}, cv::Scalar(0, 0, 150), 1);

  // 保存直线方程系数
  line_params = Eigen::Vector3d{fitting_line[0], fitting_line[1], 1.};

  return true;
}

bool LineDetector::find_line(std::vector<Eigen::Vector3d>& line_pts, Eigen::Vector3d& line_params, cv::Mat& img) const {
  img = img_ptr_->clone();
  line_pts.clear();

  // 先显示所有点
  // 范围内的点
  for (auto pt : points_) {
    int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
    // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);

    if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;

    cv::Vec3b color_value(0, 255, 0);
    img.at<cv::Vec3b>(row, col) = color_value;
  }

  // // 范围外的点
  // for (auto pt : outlier_points_) {
  //   int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
  //   // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
  //   int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);
  //
  //   if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;
  //
  //   cv::Vec3b color_value(0, 255, 0);
  //   img.at<cv::Vec3b>(row, col) = color_value;
  // }

  /// detect and get line
  // 直接从每一帧激光的正前方开始搜索一定距离范围内的符合平面标定板形状的激光线段
  // 不一定在最中间
  int n = points_.size();
  // std::cout << "mid point:" << points_.at(id).transpose() << " " << points_.at(id + 1).transpose() << std::endl;
  // 假设每个激光点之间的夹角为0.3deg,
  // step 1: 如果有激光标定板，那么激光标定板必须出现在视野的正前方 120 deg 范围内(通常相机视野也只有 120
  // deg)，也就是左右各 60deg.
  // int delta = 80 / 0.3;

  int id_left = n - 1;
  int id_right = 0;

  // int dist_left = points_.at(id_left).norm();
  // int dist_right = points_.at(id_right).norm();

  // 逻辑别搞复杂了。
  std::vector<LineSeg> segs;
  double dist_thre = 0.05;
  int skip = 3;
  int currentPt = id_right;
  int nextPt = currentPt + skip;
  bool newSeg = true;
  LineSeg seg{};
  for (int i = id_right; i < id_left - skip; i += skip) {
    if (newSeg) {
      seg.id_start = currentPt;
      seg.id_end = nextPt;
      newSeg = false;
    }

    double d1 = points_.at(currentPt).head(2).norm();
    double d2 = points_.at(nextPt).head(2).norm();

    if (fabs(d1 - d2) < dist_thre) {
      seg.id_end = nextPt;
    } else {
      newSeg = true;
      Eigen::Vector3d dist = points_.at(seg.id_start) - points_.at(seg.id_end);
      // 至少长于 20 cm, 标定板不能距离激光超过2m, 标定板上的激光点肯定多余 50 个
      if (dist.head(2).norm() > 0.2 && points_.at(seg.id_start).head(2).norm() < max_range_ &&
          points_.at(seg.id_end).head(2).norm() < max_range_ && seg.id_end - seg.id_start > 50) {
        seg.dist = dist.head(2).norm();
        segs.push_back(seg);
      }
    }

    // 下一个点
    currentPt = nextPt;
    nextPt = nextPt + skip;
  }

  // 对 segs 的边界进行扩充
  for (int i = 0; i < segs.size(); ++i) {
    LineSeg tmp = segs.at(i);

    for (int j = 1; j < 4; ++j) {
      int boundaryPt = tmp.id_end + j;
      double d1 = points_.at(tmp.id_end).head(2).norm();
      double d2 = points_.at(boundaryPt).head(2).norm();
      // 5 cm
      if (fabs(d1 - d2) < dist_thre) {
        segs.at(i).id_end = boundaryPt;
      }
    }

    for (int j = -1; j > -4; --j) {
      int boundaryPt = tmp.id_start + j;
      double d1 = points_.at(tmp.id_start).head(2).norm();
      double d2 = points_.at(boundaryPt).head(2).norm();
      //  8cm
      if (fabs(d1 - d2) < dist_thre) {
        segs.at(i).id_start = boundaryPt;
      }
    }
  }

  // 检测到直线
  if (!segs.empty()) {
    LineSeg bestLine{};
    int maxpts = -1;
    for (int i = 0; i < segs.size(); ++i) {
      LineSeg tmp = segs.at(i);
      int cnt = tmp.id_end - tmp.id_start;
      if (cnt > maxpts) {
        bestLine = tmp;
        maxpts = cnt;
      }
    }

    // 保存最佳的直线
    for (int i = bestLine.id_start; i < bestLine.id_end + 1; ++i) {
      line_pts.push_back(points_.at(i));
    }

    // 计算直线初值
    Eigen::Vector3d p1 = line_pts.front();
    Eigen::Vector3d p2 = line_pts.back();
    Eigen::Vector3d coefs;
    coefs[0] = p2.y() - p1.y();
    coefs[1] = p1.x() - p2.x();
    coefs[2] = p2.x() * p1.y() - p2.y() * p1.x();
    // std::cout << "coefs: " << coefs.transpose() << std::endl;

    // ---------------- 最小二乘拟合
    // 直线模型 Ax + By + 1 = 0
    Eigen::Vector2d fitting_line_params{coefs[0] / coefs[2], coefs[1] / coefs[2]};
    // std::cout << "init: " << fitting_line_params.transpose() << std::endl;
    if (!line_fitting_ceres(line_pts, fitting_line_params)) {
      std::cout << "fitting line failed!" << fitting_line_params.transpose() << std::endl;
      return false;
    }
    // std::cout << "fitting: " << fitting_line_params.transpose() << std::endl;

    line_params = Eigen::Vector3d{fitting_line_params[0], fitting_line_params[1], 1.};

    // 画所有直线
    // for (int j = 0; j < segs.size(); ++j) {
    //   LineSeg tmp = segs.at(j);
    //   for (int i = tmp.id_start; i < tmp.id_end; ++i) {
    //     Eigen::Vector3d pt = points_.at(i);
    //     int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
    //     int row =
    //         (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    //
    //     if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;
    //
    //     cv::Vec3b color_value(255, 0, 0);
    //     img.at<cv::Vec3b>(row, col) = color_value;
    //   }
    // }

    // 画最佳的直线
    for (int j = 0; j < line_pts.size(); ++j) {
      Eigen::Vector3d pt = line_pts.at(j);
      int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
      int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果

      if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;

      cv::Vec3b color_value(255, 0, 0);
      img.at<cv::Vec3b>(row, col) = color_value;
    }

    // ---- 画坐标
    // x轴向前，y轴向左
    int orig_col = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
    int orig_row = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);

    int x_axis_col = (int)(0.2 / img_z_ * img_focal_ + img_w_ / 2);
    int x_axis_row = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
    cv::line(img, cv::Point{orig_col, orig_row}, cv::Point{x_axis_col, x_axis_row}, cv::Scalar(255, 0, 0), 4);

    int y_axis_col = (int)(0. / img_z_ * img_focal_ + img_w_ / 2);
    int y_axis_row = (int)(-0.2 / img_z_ * img_focal_ + img_w_ / 2);

    cv::line(img, cv::Point{orig_col, orig_row}, cv::Point{y_axis_col, y_axis_row}, cv::Scalar(0, 255, 0), 4);

    return true;
  } else {
    return false;
  }
}

}  // namespace algorithm