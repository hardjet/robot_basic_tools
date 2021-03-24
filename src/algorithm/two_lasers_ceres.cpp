#include "algorithm/pose_local_parameterization.h"
#include "algorithm/two_lasers_ceres.h"

#include <utility>
#include "algorithm/util.h"

namespace algorithm {

// 共面因子
class CoPlaneFactor : public ceres::SizedCostFunction<1, 7> {
 public:
  CoPlaneFactor(Eigen::Vector3d l1, Eigen::Vector3d l2, Eigen::Vector3d c1, Eigen::Vector3d c2, double scale = 1.)
      : l1_(std::move(l1)), l2_(std::move(l2)), c1_(std::move(c1)), c2_(std::move(c2)), scale_(scale) {
    std::cout << "Co: " << l1_.transpose() << ", " << c1_.transpose() << ", " << l2_.transpose() << ", "
              << c2_.transpose() << std::endl;
    // 数据检查
    double dist = abs(l1_.x() * c1_.x() + l1_.y() * c1_.y() + l1.z()) / sqrt(l1_.x() * l1_.x() + l1_.y() * l1_.y());
    if (dist > 0.01) {
      std::cerr << "point c1 not on l1!!" << std::endl;
    }
    dist = abs(l2_.x() * c2_.x() + l2_.y() * c2_.y() + l2.z()) / sqrt(l2_.x() * l2_.x() + l2_.y() * l2_.y());
    if (dist > 0.01) {
      std::cerr << "point c2 not on l2!!" << std::endl;
    }
  }

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

  void eval(const Eigen::Vector3d &t_12, const Eigen::Quaterniond &q_12, double &residual,
            Eigen::Matrix<double, 1, 6> &jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
};

void CoPlaneFactor::eval(const Eigen::Vector3d &t_12, const Eigen::Quaterniond &q_12, double &residual,
                         Eigen::Matrix<double, 1, 6> &jacobians) const {
  Eigen::Matrix3d R_12 = q_12.toRotationMatrix();

  // EulerAngles euler = quat2euler(q_12);
  // std::cout << "q_12: " << euler << ", t_12: " << t_12.transpose() << std::endl;

  // \mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t}
  Eigen::Vector3d l3 = c1_ - R_12 * c2_ - t_12;
  // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
  Eigen::Vector3d n = skew_symmetric(l1_) * R_12 * l2_;
  // 计算残差 标量
  residual = scale_ * l3.dot(n);
  std::cout << "Co residual:" << residual << std::endl;

  // 计算雅克比
  // Eigen::Map<Eigen::Matrix<double, 1, 6>> jacobians_m(jacobians);
  // Eigen::Matrix<double, 1, 6> jaco_i;
  // 误差关于位置分量的导数 前三个
  // -(\mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2)
  Eigen::Vector3d partial_e_t = -n;
  jacobians.leftCols<3>() = partial_e_t;
  // jaco_i.leftCols<3>().setZero();
  // 误差关于角度分量的导数
  // -(\mathbf{R}\mathbf{c}^a_2) \times (\mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t})
  Eigen::Vector3d partial_e_w_1 = -skew_symmetric(R_12 * c2_) * l3;
  // - (\mathbf{R} \mathbf{l}^a_2) \times  (\mathbf{l}^a_1) \times  (\mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 -
  // \mathbf{t})
  Eigen::Vector3d partial_e_w_2 = -skew_symmetric(R_12 * l2_) * skew_symmetric(l1_) * l3;
  jacobians.rightCols<3>() = partial_e_w_1.transpose() + partial_e_w_2.transpose();

  // 传入参数使用的是四元数表示，这里面计算的实际上是关于切向量空间中的角度分量部分的导数
  // 在LocalParameterization中，ComputeJacobian已经不需要计算，这里已完全计算出雅克比
  // jacobians *= scale_;
  std::cout << "Co jacobi:" << jacobians << std::endl;
}

bool CoPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  // 参数为激光2到激光1的变换矩阵
  Eigen::Vector3d t_12(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond q_12(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Matrix3d R_12 = q_12.toRotationMatrix();

  EulerAngles euler = quat2euler(q_12);
  std::cout << "q_12: " << euler << ", t_12: " << t_12.transpose() << std::endl;

  // \mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t}
  Eigen::Vector3d l3 = c1_ - R_12 * c2_ - t_12;
  // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
  Eigen::Vector3d n = skew_symmetric(l1_) * R_12 * l2_;
  // 计算残差 标量
  residuals[0] = scale_ * l3.dot(n);
  std::cout << "Co residual:" << residuals[0] << std::endl;

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      Eigen::Matrix<double, 1, 6> jaco_i;
      // 误差关于位置分量的导数 前三个
      // -(\mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2)
      Eigen::Vector3d partial_e_t = -n;
      jaco_i.leftCols<3>() = partial_e_t;
      // jaco_i.leftCols<3>().setZero();
      // 误差关于角度分量的导数
      // -(\mathbf{R}\mathbf{c}^a_2) \times (\mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t})
      Eigen::Vector3d partial_e_w_1 = -skew_symmetric(R_12 * c2_) * l3;
      // - (\mathbf{R} \mathbf{l}^a_2) \times  (\mathbf{l}^a_1) \times  (\mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 -
      // \mathbf{t})
      Eigen::Vector3d partial_e_w_2 = -skew_symmetric(R_12 * l2_) * skew_symmetric(l1_) * l3;
      jaco_i.rightCols<3>() = partial_e_w_1.transpose() + partial_e_w_2.transpose();

      // 传入参数使用的是四元数表示，这里面计算的实际上是关于切向量空间中的角度分量部分的导数
      // 在LocalParameterization中，ComputeJacobian已经不需要计算，这里已完全计算出雅克比
      // jacobian_pose_i.leftCols<6>() = scale_ * jaco_i;
      jacobian_pose_i.leftCols<6>() = jaco_i;
      std::cout << "Co jacobi:" << jacobian_pose_i.leftCols<6>() << std::endl;
      jacobian_pose_i.rightCols<1>().setZero();
    }
  }

  return true;
}

// 共面因子
class CoPlaneErrorTerm {
 public:
  CoPlaneErrorTerm(Eigen::Vector3d l1, Eigen::Vector3d l2, Eigen::Vector3d c1, Eigen::Vector3d c2, double scale = 1.)
      : l1_(std::move(l1)), l2_(std::move(l2)), c1_(std::move(c1)), c2_(std::move(c2)), scale_(scale) {
    std::cout << "Co: " << l1_.transpose() << ", " << c1_.transpose() << ", " << l2_.transpose() << ", "
              << c2_.transpose() << std::endl;
    // 数据检查
    double dist = abs(l1_.x() * c1_.x() + l1_.y() * c1_.y() + l1.z()) / sqrt(l1_.x() * l1_.x() + l1_.y() * l1_.y());
    if (dist > 0.01) {
      std::cerr << "point c1 not on l1!!" << std::endl;
    }
    dist = abs(l2_.x() * c2_.x() + l2_.y() * c2_.y() + l2.z()) / sqrt(l2_.x() * l2_.x() + l2_.y() * l2_.y());
    if (dist > 0.01) {
      std::cerr << "point c2 not on l2!!" << std::endl;
    }
  }

  template <typename T>
  bool operator()(const T *const t_12_ptr, const T *const q_12_ptr, T *residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_12(t_12_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_12(q_12_ptr);

    const Eigen::Matrix<T, 3, 1> c1{T(c1_.x()), T(c1_.y()), T(c1_.z())};
    const Eigen::Matrix<T, 3, 1> c2{T(c2_.x()), T(c2_.y()), T(c2_.z())};
    const Eigen::Matrix<T, 3, 1> l1{T(l1_.x()), T(l1_.y()), T(l1_.z())};
    const Eigen::Matrix<T, 3, 1> l2{T(l2_.x()), T(l2_.y()), T(l2_.z())};

    Eigen::Matrix<T, 3, 3> R_12 = q_12.toRotationMatrix();

    // EulerAngles euler = quat2euler(q_12);
    // std::cout << "q_12: " << euler << ", t_12: " << t_12.transpose() << std::endl;

    // \mathbf{c}^a_1 - \mathbf{R}\mathbf{c}^a_2 - \mathbf{t}
    Eigen::Matrix<T, 3, 1> l3 = c1 - R_12 * c2 - t_12;
    // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
    Eigen::Matrix<T, 3, 1> n = skewSymmetric(l1) * R_12 * l2;
    // 计算残差 标量
    residuals_ptr[0] = scale_ * l3.dot(n);
    // std::cout << "Co residual:" << residuals_ptr[0] << std::endl;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &l1, const Eigen::Vector3d &l2, const Eigen::Vector3d &c1,
                                     const Eigen::Vector3d &c2, double scale = 1.) {
    return new ceres::AutoDiffCostFunction<CoPlaneErrorTerm, 1, 3, 4>(new CoPlaneErrorTerm(l1, l2, c1, c2, scale));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
};

// 面垂直因子
class PerpendicularPlaneFactor : public ceres::SizedCostFunction<1, 7> {
 public:
  PerpendicularPlaneFactor(Eigen::Vector3d l_1_a, Eigen::Vector3d l_1_b, Eigen::Vector3d l_2_a, Eigen::Vector3d l_2_b,
                           double scale = 1.)
      : l_1_a_(std::move(l_1_a)),
        l_1_b_(std::move(l_1_b)),
        l_2_a_(std::move(l_2_a)),
        l_2_b_(std::move(l_2_b)),
        scale_(scale) {
    std::cout << "Pp: " << l_1_a_.transpose() << ", " << l_1_b_.transpose() << ", " << l_2_a_.transpose() << ". "
              << l_2_b_.transpose() << std::endl;
  }

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

  void eval(const Eigen::Vector3d &t_12, const Eigen::Quaterniond &q_12, double &residual,
            Eigen::Matrix<double, 1, 6> &jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
};

void PerpendicularPlaneFactor::eval(const Eigen::Vector3d &t_12, const Eigen::Quaterniond &q_12, double &residual,
                                    Eigen::Matrix<double, 1, 6> &jacobians) const {
  Eigen::Matrix3d R_12 = q_12.toRotationMatrix();

  // EulerAngles euler = quat2euler(q_12);
  // std::cout << "q_12: " << euler << ", t_12: " << t_12.transpose() << std::endl;

  // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
  Eigen::Vector3d n_a = skew_symmetric(l_1_a_) * R_12 * l_2_a_;
  // \mathbf{l}^b_1 \times \mathbf{R}\mathbf{l}^b_2
  Eigen::Vector3d n_b = skew_symmetric(l_1_b_) * R_12 * l_2_b_;
  // 计算残差 标量
  residual = scale_ * n_a.dot(n_b);
  std::cout << "Pp residual:" << residual << std::endl;

  // 计算雅克比
  // Eigen::Map<Eigen::Matrix<double, 1, 6>> jacobians_m(jacobians);
  // Eigen::Matrix<double, 1, 6> jaco_i;
  // 误差关于位置分量的导数 前三个
  jacobians.leftCols<3>().setZero();
  // 误差关于角度分量的导数
  // - (\mathbf{R} \mathbf{l}^a_2) \times  (\mathbf{l}^a_1) \times (\mathbf{l}^b_1 \times \mathbf{R}\mathbf{l}^b_2)
  Eigen::Vector3d partial_e_w_1 = -skew_symmetric(R_12 * l_2_a_) * skew_symmetric(l_1_a_) * n_b;
  // - (\mathbf{R} \mathbf{l}^b_2) \times  (\mathbf{l}^b_1) \times (\mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2)
  Eigen::Vector3d partial_e_w_2 = -skew_symmetric(R_12 * l_2_b_) * skew_symmetric(l_1_b_) * n_a;
  jacobians.rightCols<3>() = partial_e_w_1.transpose() + partial_e_w_2.transpose();

  // 传入参数使用的是四元数表示，这里面计算的实际上是关于切向量空间中的角度分量部分的导数
  // 在LocalParameterization中，ComputeJacobian已经不需要计算，这里已完全计算出雅克比
  // jacobians *= scale_;
  std::cout << "Pp jacobi:" << jacobians << std::endl;
}

bool PerpendicularPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  // 参数为激光2到激光1的变换矩阵
  Eigen::Vector3d t_12(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond q_12(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Matrix3d R_12 = q_12.toRotationMatrix();

  EulerAngles euler = quat2euler(q_12);
  std::cout << "q_12: " << euler << ", t_12: " << t_12.transpose() << std::endl;

  // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
  Eigen::Vector3d n_a = skew_symmetric(l_1_a_) * R_12 * l_2_a_;
  // \mathbf{l}^b_1 \times \mathbf{R}\mathbf{l}^b_2
  Eigen::Vector3d n_b = skew_symmetric(l_1_b_) * R_12 * l_2_b_;
  // 计算残差 标量
  residuals[0] = scale_ * n_a.dot(n_b);
  std::cout << "Pp residual:" << residuals[0] << std::endl;

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
      // jacobian_pose_i.leftCols<6>() = scale_ * jaco_i;
      jacobian_pose_i.leftCols<6>() = jaco_i;
      std::cout << "Pp jacobi:" << jacobian_pose_i.leftCols<6>() << std::endl;
      jacobian_pose_i.rightCols<1>().setZero();
    }
  }

  return true;
}

// 面垂直因子
class PerpendicularPlaneErrorTerm {
 public:
  PerpendicularPlaneErrorTerm(Eigen::Vector3d l_1_a, Eigen::Vector3d l_1_b, Eigen::Vector3d l_2_a,
                              Eigen::Vector3d l_2_b, double scale = 1.)
      : l_1_a_(std::move(l_1_a)),
        l_1_b_(std::move(l_1_b)),
        l_2_a_(std::move(l_2_a)),
        l_2_b_(std::move(l_2_b)),
        scale_(scale) {
    std::cout << "Pp: " << l_1_a_.transpose() << ", " << l_1_b_.transpose() << ", " << l_2_a_.transpose() << ". "
              << l_2_b_.transpose() << std::endl;
  }

  template <typename T>
  bool operator()(const T *const t_12_ptr, const T *const q_12_ptr, T *residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_12(t_12_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_12(q_12_ptr);

    Eigen::Matrix<T, 3, 3> R_12 = q_12.toRotationMatrix();

    // EulerAngles euler = quat2euler(q_12);
    // std::cout << "q_12: " << euler << ", t_12: " << t_12.transpose() << std::endl;

    const Eigen::Matrix<T, 3, 1> l_1_a{T(l_1_a_.x()), T(l_1_a_.y()), T(l_1_a_.z())};
    const Eigen::Matrix<T, 3, 1> l_1_b{T(l_1_b_.x()), T(l_1_b_.y()), T(l_1_b_.z())};
    const Eigen::Matrix<T, 3, 1> l_2_a{T(l_2_a_.x()), T(l_2_a_.y()), T(l_2_a_.z())};
    const Eigen::Matrix<T, 3, 1> l_2_b{T(l_2_b_.x()), T(l_2_b_.y()), T(l_2_b_.z())};

    // \mathbf{l}^a_1 \times \mathbf{R}\mathbf{l}^a_2
    Eigen::Matrix<T, 3, 1> n_a = skewSymmetric(l_1_a) * R_12 * l_2_a;
    // \mathbf{l}^b_1 \times \mathbf{R}\mathbf{l}^b_2
    Eigen::Matrix<T, 3, 1> n_b = skewSymmetric(l_1_b) * R_12 * l_2_b;
    // 计算残差 标量
    residuals_ptr[0] = scale_ * n_a.dot(n_b);
    // std::cout << "Pp residual:" << residuals_ptr[0] << std::endl;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &l_1_a, const Eigen::Vector3d &l_1_b,
                                     const Eigen::Vector3d &l_2_a, const Eigen::Vector3d &l_2_b, double scale = 1.) {
    return new ceres::AutoDiffCostFunction<PerpendicularPlaneErrorTerm, 1, 3, 4>(
        new PerpendicularPlaneErrorTerm(l_1_a, l_1_b, l_2_a, l_2_b, scale));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
};

void TwoLasersCalibration(const std::vector<Observation> &obs, Eigen::Matrix4d &T12) {
  // 2到1的旋转变换
  Eigen::Quaterniond q_12(T12.block<3, 3>(0, 0));

  // 设置优化问题的待优化参数
  Eigen::VectorXd transform_12(7);
  transform_12 << T12(0, 3), T12(1, 3), T12(2, 3), q_12.x(), q_12.y(), q_12.z(), q_12.w();

  // 打印初始信息
  EulerAngles euler = quat2euler(q_12);
  std::cout << "euler_init: " << euler << std::endl;

  // 构建优化问题
  ceres::Problem problem;
  for (const auto &ob : obs) {
    // a面共勉约束
    auto *costfunction_a = new CoPlaneFactor(ob.l_1_a, ob.l_2_a, ob.c_1_a, ob.c_2_a, ob.scale);
    // b面共勉约束
    auto *costfunction_b = new CoPlaneFactor(ob.l_1_b, ob.l_2_b, ob.c_1_b, ob.c_2_b, ob.scale);
    // ab面垂直约束
    auto *costfunction_ab = new PerpendicularPlaneFactor(ob.l_1_a, ob.l_1_b, ob.l_2_a, ob.l_2_b, ob.scale);

    // ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.05 * ob.scale);
    // 添加残差块
    problem.AddResidualBlock(costfunction_a, loss_function, transform_12.data());
    problem.AddResidualBlock(costfunction_b, loss_function, transform_12.data());
    problem.AddResidualBlock(costfunction_ab, loss_function, transform_12.data());

    // problem.AddResidualBlock(costfunction_a, nullptr, transform_12.data());
    // problem.AddResidualBlock(costfunction_b, nullptr, transform_12.data());
    // problem.AddResidualBlock(costfunction_ab, nullptr, transform_12.data());
  }

  // 添加参数块
  ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
  problem.AddParameterBlock(transform_12.data(), 7, local_parameterization);

  // 设置求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // options.trust_region_strategy_type = ceres::DOGLEG;
  // options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  q_12 = Eigen::Quaterniond(transform_12[6], transform_12[3], transform_12[4], transform_12[5]);

  T12.block<3, 3>(0, 0) = q_12.toRotationMatrix();
  T12.block<3, 1>(0, 3) << transform_12[0], transform_12[1], transform_12[2];
  euler = algorithm::quat2euler(q_12);
  std::cout << "afrer opt: " << euler << std::endl;
  std::cout << "T12:\n" << T12 << std::endl;

  // // ----------------- 计算协方差
  // ceres::Covariance::Options cov_options;
  // ceres::Covariance covariance(cov_options);
  //
  // // 准备计算协方差的参数数据
  // std::vector<std::pair<const double *, const double *>> covariance_blocks;
  // covariance_blocks.emplace_back(transform_12.data(), transform_12.data());
  // // 计算当前评估参数的协方差
  // if (!covariance.Compute(covariance_blocks, &problem)) {
  //   std::cout << "covariance.Compute err!!" << std::endl;
  //   return;
  // }
  //
  // // 参数个数为7，实际有效应该是6个
  // Eigen::Matrix<double, 7, 7, Eigen::RowMajor> H_ceres = Eigen::Matrix<double, 7, 7, Eigen::RowMajor>::Zero();
  // if (!covariance.GetCovarianceBlock(transform_12.data(), transform_12.data(), H_ceres.data())) {
  //   std::cout << "covariance.GetCovarianceBlock err!!" << std::endl;
  //   return;
  // }
  // std::cout << "H_ceres:\n" << transform_12 << std::endl;
  //
  /// =============================  analysis code ==============================
  /// Get Information matrix from ceres, used to analysis the Gauge of the system
  Eigen::MatrixXd H(6, 6);
  Eigen::MatrixXd b(6, 1);
  H.setZero();
  b.setZero();
  double chi = 0;

  // 使用初始值
  // transform_12 << T12(0, 3), T12(1, 3), T12(2, 3), q_12.x(), q_12.y(), q_12.z(), q_12.w();

  for (const auto &ob : obs) {
    // a面共勉约束
    auto *costfunction_a = new CoPlaneFactor(ob.l_1_a, ob.l_2_a, ob.c_1_a, ob.c_2_a, ob.scale);
    // b面共勉约束
    auto *costfunction_b = new CoPlaneFactor(ob.l_1_b, ob.l_2_b, ob.c_1_b, ob.c_2_b, ob.scale);
    // ab面垂直约束
    auto *costfunction_ab = new PerpendicularPlaneFactor(ob.l_1_a, ob.l_1_b, ob.l_2_a, ob.l_2_b, ob.scale);

    double *residuals = new double[1];     // NOLINT
    double **jacobians = new double *[1];  // NOLINT
    jacobians[0] = new double[1 * 7];

    Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_e(jacobians[0]);
    Eigen::Map<Eigen::Matrix<double, 1, 1>> resd(residuals);

    costfunction_a->Evaluate(std::vector<double *>{transform_12.data()}.data(), residuals, jacobians);
    std::cout << "jacobian_e_a:\n" << jacobian_e << std::endl;
    H += jacobian_e.leftCols<6>().transpose() * jacobian_e.leftCols<6>();
    b -= jacobian_e.leftCols<6>().transpose() * resd;
    chi += resd * resd;

    costfunction_b->Evaluate(std::vector<double *>{transform_12.data()}.data(), residuals, jacobians);
    std::cout << "jacobian_e_b:\n" << jacobian_e << std::endl;
    H += jacobian_e.leftCols<6>().transpose() * jacobian_e.leftCols<6>();
    b -= jacobian_e.leftCols<6>().transpose() * resd;
    chi += resd * resd;

    costfunction_ab->Evaluate(std::vector<double *>{transform_12.data()}.data(), residuals, jacobians);
    std::cout << "jacobian_e_ab:\n" << jacobian_e << std::endl;
    H += jacobian_e.leftCols<6>().transpose() * jacobian_e.leftCols<6>();
    b -= jacobian_e.leftCols<6>().transpose() * resd;
    chi += resd * resd;
  }

  std::cout << "H:\n" << H << std::endl;
  std::cout << "----- H singular values--------:\n";
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  std::cout << svd.singularValues() << std::endl;
  int n = 0;
  for (size_t i = 0; i < svd.singularValues().size(); i++) {
    if (svd.singularValues()[i] < 1e-8) n++;
  }
  if (n > 0) {
    std::cout << "====== null space basis, it's means the unobservable direction for Tcl ======" << std::endl;
    std::cout << "       please note the unobservable direction is for Tcl, not for Tlc        " << std::endl;
    std::cout << svd.matrixV().rightCols(n) << std::endl;
  }

  std::cout << "\nrecover chi2: " << chi / 2. << std::endl;
}

void TwoLasersCalibrationAutoDiff(const std::vector<Observation> &obs, Eigen::Matrix4d &T12) {
  // 2到1的旋转变换
  Eigen::Quaterniond q_12(T12.block<3, 3>(0, 0));
  Eigen::Vector3d t_12(T12.block<3, 1>(0, 3));

  // 打印初始信息
  EulerAngles euler = quat2euler(q_12);
  std::cout << "euler_init: " << euler << std::endl;

  // 构建优化问题
  ceres::Problem problem;

  ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
  problem.AddParameterBlock(q_12.coeffs().data(), 4, quaternion_local_parameterization);
  // problem.AddParameterBlock(t_12.data(), 3);

  for (const auto &ob : obs) {
    // a面共勉约束
    ceres::CostFunction *cost_function_a = CoPlaneErrorTerm::Create(ob.l_1_a, ob.l_2_a, ob.c_1_a, ob.c_2_a, ob.scale);
    // b面共勉约束
    ceres::CostFunction *cost_function_b = CoPlaneErrorTerm::Create(ob.l_1_b, ob.l_2_b, ob.c_1_b, ob.c_2_b, ob.scale);
    // ab面垂直约束
    ceres::CostFunction *cost_function_ab =
        PerpendicularPlaneErrorTerm::Create(ob.l_1_a, ob.l_1_b, ob.l_2_a, ob.l_2_b, ob.scale);

    // ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.05 * ob.scale);
    // 添加残差块
    problem.AddResidualBlock(cost_function_a, loss_function, t_12.data(), q_12.coeffs().data());
    problem.AddResidualBlock(cost_function_b, loss_function, t_12.data(), q_12.coeffs().data());
    problem.AddResidualBlock(cost_function_ab, loss_function, t_12.data(), q_12.coeffs().data());
  }

  // 设置求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  T12.block<3, 3>(0, 0) = q_12.toRotationMatrix();
  T12.block<3, 1>(0, 3) = t_12;
  euler = algorithm::quat2euler(q_12);
  std::cout << "afrer opt: " << euler << std::endl;
  std::cout << "T12:\n" << T12 << std::endl;
}

void TwoLasersCalibrationNaive(const std::vector<Observation> &obs, Eigen::Matrix4d &T12) {
  // 2到1的旋转变换
  Eigen::Quaterniond q_12(T12.block<3, 3>(0, 0));
  Eigen::Vector3d t_12(T12.block<3, 1>(0, 3));

  // 打印初始信息
  EulerAngles euler = quat2euler(q_12);
  std::cout << "euler_init: " << euler << ", t:" << t_12.transpose() << std::endl;

  Eigen::Matrix<double, Eigen::Dynamic, 6> matJ(obs.size() * 3, 6);
  Eigen::Matrix<double, 6, Eigen::Dynamic> matJt(6, obs.size() * 3);
  Eigen::Matrix<double, Eigen::Dynamic, 1> f(obs.size() * 3, 1);
  Eigen::Matrix<double, 6, 6> matJtJ;
  Eigen::Matrix<double, 6, 1> Jtf;
  Eigen::Matrix<double, 6, 1> delta_x;

  matJ.setZero();
  f.setZero();
  size_t i = 0;
  for (const auto &ob : obs) {
    // a面共勉约束
    CoPlaneFactor co_a(ob.l_1_a, ob.l_2_a, ob.c_1_a, ob.c_2_a, ob.scale);
    Eigen::Matrix<double, 1, 6> jaco_a;
    co_a.eval(t_12, q_12, f(i * 3), jaco_a);
    matJ.block<1, 6>(i * 3, 0) = jaco_a;

    // b面共勉约束
    CoPlaneFactor co_b(ob.l_1_b, ob.l_2_b, ob.c_1_b, ob.c_2_b, ob.scale);
    Eigen::Matrix<double, 1, 6> jaco_b;
    co_b.eval(t_12, q_12, f(i * 3 + 1), jaco_b);
    matJ.block<1, 6>(i * 3 + 1, 0) = jaco_b;

    // ab面垂直约束
    PerpendicularPlaneFactor co_ab(ob.l_1_a, ob.l_1_b, ob.l_2_a, ob.l_2_b, ob.scale);
    Eigen::Matrix<double, 1, 6> jaco_ab;
    co_ab.eval(t_12, q_12, f(i * 3 + 2), jaco_ab);
    matJ.block<1, 6>(i * 3 + 2, 0) = jaco_ab;

    i++;
  }

  std::cout << "matJ:\n" << matJ << std::endl;
  std::cout << "f.transpose():\n" << f.transpose() << std::endl;

  matJt = matJ.transpose();
  matJtJ = matJt * matJ;

  std::cout << "matJtJ:\n" << matJtJ << std::endl;

  Jtf = matJt * f;
  std::cout << "-Jtf:\n" << -Jtf.transpose() << std::endl;

  // J^t * J * delta_x = -J^t * f
  delta_x = matJtJ.colPivHouseholderQr().solve(-Jtf);

  std::cout << "delta_x:\n" << delta_x.transpose() << std::endl;

  std::cout << "matJtJ*delta_x:\n" << matJtJ * delta_x << std::endl;

  // std::cout << "----- matJtJ singular values--------:\n";
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd(matJtJ, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // std::cout << svd.singularValues() << std::endl;

}

}  // namespace algorithm
