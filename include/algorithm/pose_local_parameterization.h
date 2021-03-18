#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"

namespace algorithm {

class PoseLocalParameterization : public ceres::LocalParameterization {
  bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
  bool ComputeJacobian(const double *x, double *jacobian) const override;
  int GlobalSize() const override { return 7; };
  int LocalSize() const override { return 6; };
};

Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta) {
  Eigen::Quaterniond dq;
  Eigen::Vector3d half_theta = theta;
  half_theta /= static_cast<double>(2.0);
  dq.w() = static_cast<double>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();
  return true;
}

// 实际的雅克比矩阵在costfunc中已经计算完毕，因此这里直接设置为单位矩阵
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

}  // namespace algorithm
