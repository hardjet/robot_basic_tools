#pragma once

#include <Eigen/Core>

namespace algorithm {

struct Observation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 激光1在a面上线段的单位向量
  Eigen::Vector3d l_1_a;
  // 激光1在a面上线段中点
  Eigen::Vector3d c_1_a;
  // 激光1在b面上线段的单位向量
  Eigen::Vector3d l_1_b;
  // 激光1在b面上线段中点
  Eigen::Vector3d c_1_b;
  // 激光2在a面上线段的单位向量
  Eigen::Vector3d l_2_a;
  // 激光2在a面上线段中点
  Eigen::Vector3d c_2_a;
  // 激光2在b面上线段的单位向量
  Eigen::Vector3d l_2_b;
  // 激光2在b面上线段中点
  Eigen::Vector3d c_2_b;
  // 权重
  double scale{1.0};
};

/**
 * 使用ceres求解，自己构建梯度求解以及LocalParameterization
 * @param obs 观测量集合
 * @param T 激光2到1的变换矩阵
 */
void TwoLasersCalibration(const std::vector<Observation> &obs, Eigen::Matrix4d &T12, bool is_tx_fixed);

/**
 * 使用ceres求解，使用AutoDiff以及ceres::EigenQuaternionParameterization
 * @param obs 观测量集合
 * @param T 激光2到1的变换矩阵
 */
void TwoLasersCalibrationAutoDiff(const std::vector<Observation> &obs, Eigen::Matrix4d &T12);

/**
 * 原生解法
 * @param obs 观测量集合
 * @param T 激光2到1的变换矩阵
 */
void TwoLasersCalibrationNaive(const std::vector<Observation> &obs, Eigen::Matrix4d &T12);

}  // namespace algorithm
