#include "ceres/ceres.h"

#include "camera_model/camera_models/Camera.h"
#include "camera_model/camera_models/CostFunctionFactory.h"
#include "camera_model/gpl/EigenQuaternionParameterization.h"
#include "camera_model/gpl/EigenUtils.h"
#include "camera_model/sparse_graph/Transform.h"

#include "algorithm/util.h"
#include "algorithm/two_cameras_ceres.h"

namespace algorithm {
namespace TwoCamerasCalib {

bool calibrate(const boost::shared_ptr<camera_model::Camera> &camera_ptr, const Observation &obs,
               Eigen::Matrix4d &T_21) {
  // 1到2的旋转变换
  // Eigen::Quaterniond q_21(T_21.block<3, 3>(0, 0));

  // 打印初始信息
  // EulerAngles euler = quat2euler(q_21);
  // std::cout << "euler_init: " << euler << std::endl;
  // std::cout << "T_21_init: \n" << T_21 << std::endl;

  camera_model::Transform transform_21(T_21);

  // 构建优化问题
  ceres::Problem problem;
  for (uint i = 0; i < obs.object_points.size(); i++) {
    ceres::CostFunction *costFunction = camera_model::CostFunctionFactory::instance()->generateCostFunction(
        camera_ptr, obs.object_points.at(i), obs.image_points.at(i), camera_model::CAMERA_POSE);

    ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
    problem.AddResidualBlock(costFunction, lossFunction, transform_21.rotationData(), transform_21.translationData());
  }

  // 添加参数块
  ceres::LocalParameterization *quaternionParameterization = new camera_model::EigenQuaternionParameterization;
  problem.SetParameterization(transform_21.rotationData(), quaternionParameterization);

  // 设置求解器
  ceres::Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  options.num_threads = 4;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  // options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // std::cout << summary.FullReport() << std::endl;

  T_21 = transform_21.toMatrix();
  // euler = algorithm::quat2euler(transform_21.rotation());
  // std::cout << "afrer opt: " << euler << std::endl;
  // std::cout << "T_21:\n" << T_21 << std::endl;

  return summary.IsSolutionUsable();
}
}  // namespace TwoCamerasCalib
}  // namespace algorithm
