#include "algorithm/pose_local_parameterization.h"
#include "algorithm/two_lasers_ceres.h"
#include "algorithm/util.h"

namespace algorithm {

// 共面因子
class CoPlaneFactor : public ceres::SizedCostFunction<1, 7> {
 private:
  // 线段1单位向量
  const Eigen::Vector3d l1_;
  // 线段1的中点坐标
  const Eigen::Vector3d c1_;
  // 线段2单位向量
  const Eigen::Vector3d l2_;
  // 线段2的中点坐标
  const Eigen::Vector3d c2_;
  const double scale_;

 public:
  CoPlaneFactor(Eigen::Vector3d &l1, Eigen::Vector3d &l2, Eigen::Vector3d &c1, Eigen::Vector3d &c2, double scale = 1.)
      : l1_(l1), l2_(l2), c1_(c1), c2_(c2), scale_(scale) {}

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

bool CoPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  // 参数为激光2到激光1的变换矩阵
  Eigen::Vector3d t_12(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond q_12(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Matrix3d R_12 = q_12.toRotationMatrix();

  // \mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t}
  Eigen::Vector3d l3 = c1_ - R_12 * c2_ - t_12;
  // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
  Eigen::Vector3d n3 = skew_symmetric(l1_) * R_12 * l2_;
  // 计算残差 标量
  residuals[0] = l3.dot(n3);
  // std::cout << residuals[0] <<std::endl;

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      Eigen::Matrix<double, 1, 6> jaco_i;
      // 误差关于位置分量的导数 前三个
      // -(\mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2)
      Eigen::Vector3d partial_e_t = -n3;
      jaco_i.leftCols<3>() = partial_e_t;
      // 误差关于角度分量的导数
      // -(\mathbf{R}\mathbf{c}^a_2) \times (\mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t})
      Eigen::Vector3d partial_e_w_1 = -skew_symmetric(R_12 * c2_) * l3;
      // - (\mathbf{R} \mathbf{l}^a_2) \times  (\mathbf{l}^a_1) \times  (\mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 -
      // \mathbf{t})
      Eigen::Vector3d partial_e_w_2 = -skew_symmetric(R_12 * l2_) * skew_symmetric(l1_) * l3;
      jaco_i.rightCols<3>() = partial_e_w_1.transpose() + partial_e_w_2.transpose();

      // 传入参数使用的是四元数表示，这里面计算的实际上是关于切向量空间中的角度分量部分的导数
      // 在LocalParameterization中，ComputeJacobian已经不需要计算，这里已完全计算出雅克比
      jacobian_pose_i.leftCols<6>() = scale_ * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }
  }

  return true;
}

// 面垂直因子
class PerpendicularPlaneFactor : public ceres::SizedCostFunction<1, 7> {
 private:
  // 激光1在a面上线段的单位向量
  const Eigen::Vector3d l_1_a_;
  // 激光1在b面上线段的单位向量
  const Eigen::Vector3d l_1_b_;
  // 激光2在a面上线段的单位向量
  const Eigen::Vector3d l_2_a_;
  // 激光2在b面上线段的单位向量
  const Eigen::Vector3d l_2_b_;
  // 权重
  const double scale_;

 public:
  PerpendicularPlaneFactor(Eigen::Vector3d &l_1_a, Eigen::Vector3d &l_1_b, Eigen::Vector3d &l_2_a,
                           Eigen::Vector3d &l_2_b, double scale = 1.)
      : l_1_a_(l_1_a), l_1_b_(l_1_b), l_2_a_(l_2_a), l_2_b_(l_2_b), scale_(scale) {}

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

bool PerpendicularPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  // 参数为激光2到激光1的变换矩阵
  Eigen::Vector3d t_12(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond q_12(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Matrix3d R_12 = q_12.toRotationMatrix();

  // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
  Eigen::Vector3d n_a = skew_symmetric(l_1_a_) * R_12 * l_2_a_;
  // \mathbf{l}^b_1 \times \mathbf{R}\mathbf{l}^b_2
  Eigen::Vector3d n_b = skew_symmetric(l_1_b_) * R_12 * l_2_b_;
  // 计算残差 标量
  residuals[0] = n_a.dot(n_b);
  // std::cout << residuals[0] <<std::endl;

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      Eigen::Matrix<double, 1, 6> jaco_i;
      // 误差关于位置分量的导数 前三个
      jaco_i.leftCols<3>().setZero();
      // 误差关于角度分量的导数
      // - (\mathbf{R} \mathbf{l}^a_2) \times  (\mathbf{l}^a_1) \times (\mathbf{l}^b_1 \times \mathbf{R}\mathbf{l}^b_2)
      Eigen::Vector3d partial_e_w_1 = -skew_symmetric(R_12 * l_2_a_) * skew_symmetric(l_1_a_) * n_b;
      // - (\mathbf{R} \mathbf{l}^b_2) \times  (\mathbf{l}^b_1) \times (\mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2)
      Eigen::Vector3d partial_e_w_2 = -skew_symmetric(R_12 * l_2_b_) * skew_symmetric(l_1_b_) * n_a;
      jaco_i.rightCols<3>() = partial_e_w_1.transpose() + partial_e_w_2.transpose();

      // 传入参数使用的是四元数表示，这里面计算的实际上是关于切向量空间中的角度分量部分的导数
      // 在LocalParameterization中，ComputeJacobian已经不需要计算，这里已完全计算出雅克比
      jacobian_pose_i.leftCols<6>() = scale_ * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }
  }

  return true;
}

void TwoLasersCalibration(const std::vector<Observation> &obs, Eigen::Matrix4d &T12) {}

}  // namespace algorithm
