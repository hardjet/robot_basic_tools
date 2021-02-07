#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace algorithm {

struct EulerAngles {
  double yaw, pitch, roll;
};

EulerAngles ToEulerAngles(const Eigen::Quaterniond& q);

Eigen::Vector4d pi_from_ppp(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3);

}  // namespace algorithm
