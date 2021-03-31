#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace algorithm {

#define DEG2RAD_RBT(x) (x * M_PI / 180.0)
#define RAD2DEG_RBT(x) (x * 180.0 / M_PI)

struct EulerAngles {
  double yaw{0.};
  double pitch{0.};
  double roll{0.};
  friend std::ostream& operator<<(std::ostream&, const EulerAngles&);
};

/// 切空间到se3
Eigen::Matrix4d lie_to_se3(const Eigen::Matrix<double, 6, 1>& lie);

/// ZYX顺序转为四元数
Eigen::Quaterniond ypr2quaternion(double yaw, double pitch, double roll);

/// 四元数转欧拉角
EulerAngles quat2euler(const Eigen::Quaterniond& q);

/// 3个点确定一个平面
Eigen::Vector4d plane_from_3pts(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3);

/// 直线拟合
void line_fitting_ceres(const std::vector<Eigen::Vector3d>& points, Eigen::Vector2d& line_params);

/// 计算斜对称矩阵 reference to gtsam
/**
 * skew symmetric matrix returns this:
 *   0  -wz   wy
 *  wz    0  -wx
 * -wy   wx    0
 * @param wx 3 dimensional vector
 * @param wy
 * @param wz
 * @return a 3*3 skew symmetric matrix
 */
inline Eigen::Matrix3d skew_symmetric(double wx, double wy, double wz) {
  return (Eigen::Matrix3d() << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0).finished();
}

/// 计算斜对称矩阵
template <class Derived>
inline Eigen::Matrix3d skew_symmetric(const Eigen::MatrixBase<Derived>& w) {
  return skew_symmetric(w(0), w(1), w(2));
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived>& m) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> skew;
  skew << typename Derived::Scalar(0), -m(2), m(1), m(2), typename Derived::Scalar(0), -m(0), -m(1), m(0),
      typename Derived::Scalar(0);
  return skew;
}

void remove_cols(Eigen::MatrixXd& data, const std::vector<size_t>& idx_to_remove);

}  // namespace algorithm
