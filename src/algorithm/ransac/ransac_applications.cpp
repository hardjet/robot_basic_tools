/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "algorithm/ransac/ransac_applications.h"
#include "algorithm/util.h"

namespace algorithm {
namespace ransac {

/*---------------------------------------------------------------
                Aux. functions needed by ransac_detect_2D_lines
 ---------------------------------------------------------------*/

inline void ransac2Dline_fit(const Eigen::MatrixXd& allData, const std::vector<size_t>& useIndices,
                             std::vector<Eigen::MatrixXd>& fitModels) {
  assert(useIndices.size() == 2);

  Eigen::Vector2d p1(allData(0, useIndices[0]), allData(1, useIndices[0]));
  Eigen::Vector2d p2(allData(0, useIndices[1]), allData(1, useIndices[1]));

  try {
    Eigen::Vector3d line;
    if (p1 == p2) throw std::logic_error("Both points are the same");
    line[0] = p2.y() - p1.y();
    line[1] = p1.x() - p2.x();
    line[2] = p2.x() * p1.y() - p2.y() * p1.x();

    // std::cout << "fit: " << line.transpose() << std::endl;

    // if(abs(line[0]) < 1e-4 || abs(line[1]) < 1e-4 || abs(line[2]) < 1e-4) {
    //   std::cout << "p1:" << p1.transpose() << std::endl;
    //   std::cout << "p2:" << p2.transpose() << std::endl;
    // }

    fitModels.resize(1);
    Eigen::MatrixXd& M = fitModels[0];

    M.resize(1, 3);
    for (size_t i = 0; i < 3; i++) M(0, i) = static_cast<double>(line[i]);
  } catch (std::exception&) {
    fitModels.clear();
    return;
  }
}

inline void ransac2Dline_distance(const Eigen::MatrixXd& allData, const std::vector<Eigen::MatrixXd>& testModels,
                                  const double distanceThreshold, unsigned int& out_bestModelIndex,
                                  std::vector<size_t>& out_inlierIndices) {
  out_inlierIndices.clear();
  out_bestModelIndex = 0;

  if (testModels.empty()) return;  // No model, no inliers.

  assert(testModels.size() == 1);
  const Eigen::MatrixXd& M = testModels[0];

  assert(M.rows() == 1 && M.cols() == 3);

  // Ax + By + C = 0
  Eigen::Vector3d line;
  line[0] = M(0, 0);
  line[1] = M(0, 1);
  line[2] = M(0, 2);

  // 计算点到直线的距离
  auto distance = [&line](double x, double y) {
    return std::abs(line[0] * x + line[1] * y + line[2]) / sqrt(line[0] * line[0] + line[1] * line[1]);
  };

  const size_t N = allData.cols();
  out_inlierIndices.reserve(100);
  for (size_t i = 0; i < N; i++) {
    const double d = distance(double(allData(0, i)), double(allData(1, i)));
    if (d < distanceThreshold) out_inlierIndices.push_back(i);
  }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
 */
bool ransac2Dline_degenerate(const Eigen::MatrixXd& allData, const std::vector<size_t>& useIndices) { return false; }

/*---------------------------------------------------------------
                                ransac_detect_2D_lines
 ---------------------------------------------------------------*/
void ransac_detect_2D_lines(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                            std::vector<std::pair<size_t, Eigen::Vector3d>>& out_detected_lines, double threshold,
                            size_t min_inliers_for_valid_line = 5) {
  assert(x.size() == y.size());
  out_detected_lines.clear();

  if (x.size() == 0) {
    printf("data empty!!\n");
    return;
  }

  // The running lists of remaining points after each plane, as a matrix:
  Eigen::MatrixXd remainingPoints(2, x.size());
  for (size_t i = 0; i < x.size(); i++) {
    remainingPoints(0, i) = x(i);
    remainingPoints(1, i) = y(i);
  }

  //   for (int i = 0; i < x.size();) {
  //     printf("remainingPoints[%d]:%f, %f\n", i, remainingPoints(0, i), remainingPoints(1, i));
  //     i += 10;
  //   }

  // ---------------------------------------------
  // For each line:
  // ---------------------------------------------
  int max_try = 0;
  while (remainingPoints.cols() >= 2) {
    std::vector<size_t> this_best_inliers;
    Eigen::MatrixXd this_best_model;

    RANSAC ransac;
    ransac.execute(remainingPoints, ransac2Dline_fit,
                   ransac2Dline_distance,    //
                   ransac2Dline_degenerate,  //
                   threshold,                //
                   2,                        // Minimum set of points
                   this_best_inliers, this_best_model,
                   0.99999  // Prob. of good result
    );

    // Is this plane good enough?
    //    printf("----- ransac_detect_2D_lines() ..... best inliers count = %zu\n", this_best_inliers.size());
    if (this_best_inliers.size() >= min_inliers_for_valid_line) {
      // Add this plane to the output list:
      out_detected_lines.emplace_back(
          this_best_inliers.size(),
          Eigen::Vector3d(double(this_best_model(0, 0)), double(this_best_model(0, 1)), double(this_best_model(0, 2))));

      // 归一化
      auto l = out_detected_lines.back().second;
      double s = sqrt(l[0] * l[0] + l[1] * l[1]);
      l /= s;
      out_detected_lines.back().second = l;
      // std::cout << "line model:" << l.transpose() << std::endl;

      // Discard the selected points so they are not used again for
      // finding subsequent planes:
      // std::cout << "before remove_cols:" << remainingPoints.cols() << std::endl;
      remove_cols(remainingPoints, this_best_inliers);
      // std::cout << "after remove_cols:" << remainingPoints.cols() << std::endl;
      // std::cout << "out_detected_lines:" << out_detected_lines.size() << std::endl;

      // if (remainingPoints.cols()) {
      //   for (int i = 0; i < remainingPoints.cols();) {
      //     printf("remainingPoints[%d]:%f, %f\n", i, remainingPoints(0, i), remainingPoints(1, i));
      //     i += 10;
      //   }
      // }

      max_try++;
      if (max_try == 10) {
        break;
      }
    } else {
      break;  // Do not search for more planes.
    }
  }
}
}  // namespace ransac
}  // namespace algorithm
