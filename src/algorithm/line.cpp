#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc.hpp>

#include "algorithm/line.hpp"

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
  best_line_pts_.resize(0);
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

bool Line::find_line(cv::Mat& img) {
  img = img_ptr_->clone();

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

  // 范围外的点
  for (auto pt : outlier_points_) {
    int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
    // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
    int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);

    if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;

    cv::Vec3b color_value(0, 255, 0);
    img.at<cv::Vec3b>(row, col) = color_value;
  }

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
      if (dist.head(2).norm() > 0.2 && points_.at(seg.id_start).head(2).norm() < 2 &&
          points_.at(seg.id_end).head(2).norm() < 2 && seg.id_end - seg.id_start > 50) {
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

    for (int i = bestLine.id_start; i < bestLine.id_end + 1; ++i) {
      best_line_pts_.push_back(points_.at(i));
    }

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

    for (int j = 0; j < best_line_pts_.size(); ++j) {
      Eigen::Vector3d pt = best_line_pts_.at(j);
      int col = (int)(pt.x() / img_z_ * img_focal_ + img_w_ / 2);
      int row = (int)(-pt.y() / img_z_ * img_focal_ + img_w_ / 2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果

      if (col > img_w_ - 1 || col < 0 || row > img_w_ - 1 || row < 0) continue;

      cv::Vec3b color_value(255, 0, 0);
      img.at<cv::Vec3b>(row, col) = color_value;
    }

    cv::putText(img, "Detecting the Laser Points on the calibra planar!", cv::Point(5, 30),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    return true;
  } else {
    return false;
  }
}

}  // namespace algorithm