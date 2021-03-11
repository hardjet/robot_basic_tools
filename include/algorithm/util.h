#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace algorithm {

#define DEG2RAD(x) (x * M_PI / 180.0)
#define RAD2DEG(x) (x * 180.0 / M_PI)

struct EulerAngles {
  double yaw, pitch, roll;
};

EulerAngles ToEulerAngles(const Eigen::Quaterniond& q);

Eigen::Vector4d pi_from_ppp(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3);

}  // namespace algorithm
