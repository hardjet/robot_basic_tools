#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace algorithm {

#define DEG2RAD_RBT(x) (x * M_PI / 180.0)
#define RAD2DEG_RBT(x) (x * 180.0 / M_PI)

struct EulerAngles {
  double yaw, pitch, roll;
};

EulerAngles ToEulerAngles(const Eigen::Quaterniond& q);

Eigen::Vector4d pi_from_ppp(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3);

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q) {
  Eigen::Matrix3d ans;
  ans << 0.0, -q(2), q(1), q(2), 0.0, -q(0), -q(1), q(0), 0.0;
  return ans;
}

}  // namespace algorithm
