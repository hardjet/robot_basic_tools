// #include <Eigen/Eigenvalues>

#include "algorithm/util.h"

namespace algorithm {

std::ostream& operator<<(std::ostream& os, const EulerAngles& euler) {
  os << "[deg] roll: " << RAD2DEG_RBT(euler.roll) << ", pitch: " << RAD2DEG_RBT(euler.pitch)
     << ", yaw: " << RAD2DEG_RBT(euler.yaw);
  return os;
}

/**
 * converts a number constant to a number_t constant at compile time
 * to avoid having to cast everything to avoid warnings.
 **/
inline constexpr double cst(long double v) { return (double)v; }

/**
 *
 * @param lie 旋转部分在前 平移部分在后
 * @return
 */
Eigen::Matrix4d lie_to_se3(const Eigen::Matrix<double, 6, 1>& lie) {
  Eigen::Vector3d omega;
  for (int i = 0; i < 3; i++) omega[i] = lie[i];
  Eigen::Vector3d upsilon;
  for (int i = 0; i < 3; i++) upsilon[i] = lie[i + 3];

  double theta = omega.norm();
  Eigen::Matrix3d Omega = skew_symmetric(omega);

  Eigen::Matrix3d R;
  Eigen::Matrix3d V;
  if (theta < cst(0.00001)) {
    Eigen::Matrix3d Omega2 = Omega * Omega;

    R = (Eigen::Matrix3d::Identity() + Omega + cst(0.5) * Omega2);

    V = (Eigen::Matrix3d::Identity() + cst(0.5) * Omega + cst(1.) / cst(6.) * Omega2);
  } else {
    Eigen::Matrix3d Omega2 = Omega * Omega;

    R = (Eigen::Matrix3d::Identity() + std::sin(theta) / theta * Omega +
         (1 - std::cos(theta)) / (theta * theta) * Omega2);

    V = (Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * Omega +
         (theta - std::sin(theta)) / (std::pow(theta, 3)) * Omega2);
  }

  Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
  se3.block<3, 3>(0, 0) = R;
  se3.block<3, 1>(0, 3) = V * upsilon;

  return se3;
}

// yaw (Z), pitch (Y), roll (X)
Eigen::Quaterniond ypr2quat(double yaw, double pitch, double roll) {
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Eigen::Quaterniond q;
  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;

  return q;
}

EulerAngles quat2euler(const Eigen::Quaterniond& q) {
  EulerAngles angles{};

  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    angles.pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles.pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}

/*
 三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
 (a,b,c)^T = vector(AO) cross vector(BO)
 d = O.dot(cross(AO,BO))
 */
Eigen::Vector4d plane_from_3pts(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3) {
  Eigen::Vector4d plane;
  plane << (x1 - x3).cross(x2 - x3),
      -x3.dot(x1.cross(x2));  // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

  return plane;
}

/**
 * 求直线与平面的交点p
 * @param plane_normal 平面法向量
 * @param p_on_plane 平面上的一个点
 * @param line_vector 直线方向向量
 * @param p_on_line 直线上的一个点
 * @param p 直线与平面的交点
 * @return 是否有一个交点
 */
bool plane_line_intersect_point(const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& p_on_plane,
                                const Eigen::Vector3d& line_vector, const Eigen::Vector3d& p_on_line,
                                Eigen::Vector3d& p) {
  // 判断直线与平面是否平行
  double r = plane_normal.transpose() * line_vector;
  if (fabs(r) < 1e-6) {
    return false;
  }

  double t = (p_on_plane - p_on_line).transpose() * plane_normal;
  t /= r;
  p = p_on_line + t * line_vector;
  return true;
}

void remove_cols(Eigen::MatrixXd& data, const std::vector<size_t>& idx_to_remove) {
  std::vector<std::size_t> idxs = idx_to_remove;
  std::sort(idxs.begin(), idxs.end());
  auto itEnd = std::unique(idxs.begin(), idxs.end());
  idxs.resize(itEnd - idxs.begin());

  Eigen::MatrixXd data_after_remove(2, data.cols());

  // idx_to_remove cnt
  size_t idx = 0;
  // data_after_remove cnt
  long cnt = 0;
  for (long i = 0; i < data.cols(); i++) {
    if (i != idxs[idx]) {
      data_after_remove.block<2, 1>(0, cnt) = data.block<2, 1>(0, i);
      cnt++;
    } else {
      idx++;
    }
  }

  Eigen::MatrixXd tmp(2, data.cols() - idxs.size());
  tmp = data_after_remove.block(0, 0, 2, tmp.cols());
  data.swap(tmp);

  // printf("data size:[%ld], cnt = %zu, idx = %zu\n", data.cols(), cnt, idx);
}

}  // namespace algorithm
