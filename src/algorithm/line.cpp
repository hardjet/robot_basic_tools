#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc.hpp>

#include "algorithm/line.h"

namespace algorithm {

#define DEG2RAD(x) (x * M_PI / 180.0)

struct LineSeg {
  int id_start;
  int id_end;
  double dist;
};

Line::Line(const sensor_msgs::LaserScan& scan, double angle_range, double max_range)
    : angle_range_(DEG2RAD(angle_range)), max_range_(max_range) {
  points_.resize(0);
  outlier_points_.resize(0);
  img_ptr_ = std::make_shared<cv::Mat>(img_w_, img_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  // 转换点
  scan2points(scan);
}

void Line::scan2points(const sensor_msgs::LaserScan& scan_in) {
  size_t n_pts = scan_in.ranges.size();

  double angle = scan_in.angle_min;
  double x, y;
  for (size_t i = 0; i < n_pts; ++i) {
    x = scan_in.ranges[i] * cos(angle);
    y = scan_in.ranges[i] * sin(angle);
    // 范围内的点
    if (angle > -angle_range_ && angle < angle_range_ && scan_in.ranges[i] < max_range_) {
      points_.emplace_back(x, y, 0);
    } else {
      outlier_points_.emplace_back(x, y, 0);
    }

    // 角增量
    angle += scan_in.angle_increment;
  }
}

bool Line::find_line(std::vector<Eigen::Vector3d>& best_line_pts, cv::Mat& img) {
  img = img_ptr_->clone();
  best_line_pts.clear();

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
      best_line_pts.push_back(points_.at(i));
    }

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
    for (int j = 0; j < best_line_pts.size(); ++j) {
      Eigen::Vector3d pt = best_line_pts.at(j);
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