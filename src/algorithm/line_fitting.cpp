#include <iostream>

#include "ceres/ceres.h"
#include "algorithm/util.h"

namespace algorithm {

struct LineFittingResidfual {
  LineFittingResidfual(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T *const m, T *residual) const {
    residual[0] = m[0] * T(x_) + m[1] * T(y_) + T(1.);
    return true;
  }

 private:
  // Observations for a sample.
  const double x_;
  const double y_;
};

/**
 *
 * @param points
 * @param line_params 直线模型 Ax + By + 1 = 0
 */
bool line_fitting_ceres(const std::vector<Eigen::Vector3d> &points, Eigen::Vector2d &line_params) {
  double line[2] = {line_params(0), line_params(1)};

  ceres::Problem problem;
  for (auto obi : points) {
    // debug
    // if (i % 5 == 0) {
    //   std::cout << obi.transpose() << std::endl;
    // }

    ceres::CostFunction *costfunction =
        new ceres::AutoDiffCostFunction<LineFittingResidfual, 1, 2>(new LineFittingResidfual(obi.x(), obi.y()));

    // ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.05);
    problem.AddResidualBlock(costfunction, loss_function, line);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 20;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // std::cout << summary.FullReport() << std::endl;

  line_params(0) = line[0];
  line_params(1) = line[1];

  return summary.IsSolutionUsable();
}

}  // namespace algorithm
