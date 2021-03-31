/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include "algorithm/ransac/ransac.h"

namespace algorithm {
namespace ransac {

/** Fit a number of 2-D lines to a given point cloud, automatically determining
 * the number of existing lines by means of the provided threshold and minimum
 * number of supporting inliers.
 * \param out_detected_lines The output list of pairs: number of supporting
 * inliers, detected line.
 * \param threshold The maximum distance between a point and a temptative line
 * such as the point is considered an inlier.
 * \param min_inliers_for_valid_line  The minimum number of supporting inliers
 * to consider a line as valid.
 */

void ransac_detect_2D_lines(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                            std::vector<std::pair<size_t, Eigen::Vector3d>>& out_detected_lines, double threshold,
                            size_t min_inliers_for_valid_line);

}  // namespace ransac
}  // namespace algorithm
