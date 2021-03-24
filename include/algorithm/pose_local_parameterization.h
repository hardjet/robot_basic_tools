#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "algorithm/util.h"

namespace algorithm {

class PoseLocalParameterization : public ceres::LocalParameterization {
  // x是7维数据，delta是切空间的增量6维，x_plus_delta是7维数据
  bool Plus(const double *x, const double *delta, double *x_plus_delta) const override {
    // 位置分量
    Eigen::Map<const Eigen::Vector3d> x_t(x);
    // 角度分量 内部内存排序为[x,y,z,w]
    Eigen::Map<const Eigen::Quaterniond> x_q(x + 3);

    // 切空间位置增量
    Eigen::Map<const Eigen::Vector3d> delta_t(delta);
    // 注意这里需要将切空间角度增量变换为四元数增量
    Eigen::Quaterniond delta_q = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));
    std::cout << "delta_t: " << delta_t.transpose()
              << ", delta_q: " << Eigen::Map<const Eigen::Vector3d>(delta + 3).transpose() << std::endl;

    EulerAngles delta_euler = quat2euler(delta_q);

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = x_t + delta_t;
    q = (x_q * delta_q).normalized();

    EulerAngles euler = quat2euler(q);
    std::cout << "delta_euler: " << delta_euler << ", q_euler: " << euler << std::endl;

    return true;
  };

  // bool Plus(const double *x, const double *delta, double *x_plus_delta) const override {
  //   // 位置分量
  //   Eigen::Map<const Eigen::Vector3d> x_t(x);
  //   // 角度分量 内部内存排序为[x,y,z,w]
  //   Eigen::Map<const Eigen::Quaterniond> x_q(x + 3);
  //
  //   // 切空间位置增量
  //   Eigen::Map<const Eigen::Vector3d> delta_t(delta);
  //   // 注意这里需要将切空间角度增量变换为四元数增量
  //   Eigen::Quaterniond delta_q = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));
  //   std::cout << "delta_t: " << delta_t.transpose()
  //             << ", delta_q: " << Eigen::Map<const Eigen::Vector3d>(delta + 3).transpose() << std::endl;
  //
  //   EulerAngles delta_euler = quat2euler(delta_q);
  //
  //   Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  //
  //   p = x_t + delta_t;
  //   // q = (x_q * delta_q).normalized();
  //
  //   const double norm_delta = sqrt(delta[3] * delta[3] + delta[4] * delta[4] + delta[5] * delta[5]);
  //   if (norm_delta > 0.0) {
  //     const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
  //     double q_delta[4];
  //     q_delta[0] = cos(norm_delta);
  //     q_delta[1] = sin_delta_by_delta * delta[3];
  //     q_delta[2] = sin_delta_by_delta * delta[4];
  //     q_delta[3] = sin_delta_by_delta * delta[5];
  //     ceres::QuaternionProduct(q_delta, x + 3, x_plus_delta + 3);
  //   } else {
  //     for (int i = 0; i < 4; ++i) {
  //       x_plus_delta[i + 3] = x[i + 3];
  //     }
  //   }
  //
  //   Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);
  //   EulerAngles euler = quat2euler(q);
  //   std::cout << "delta_euler: " << delta_euler << ", q_euler: " << euler << std::endl;
  //
  //   return true;
  // };

  // 实际的雅克比矩阵在costfunc中已经计算完毕，因此这里直接设置为单位矩阵
  bool ComputeJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
  };

  static Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta) {
    Eigen::Quaterniond dq;
    Eigen::Vector3d half_theta = theta;
    half_theta /= 2.0;
    dq.w() = 1.0;
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  int GlobalSize() const override { return 7; };
  int LocalSize() const override { return 6; };
};

}  // namespace algorithm
