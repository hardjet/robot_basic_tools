// reference to mrpt

#pragma once

#include <Eigen/Core>

#include <vector>
#include <functional>
#include <set>

namespace algorithm {
namespace ransac {
/** @addtogroup ransac_grp RANSAC and other model fitting algorithms
 * \ingroup mrpt_math_grp
 * @{ */

/** Define overloaded functions for user types as required.
 * This default implementation assumes datasets in matrices, with each sample
 * being a column, the dimensionality being the number of rows. */
template <typename Derived>
size_t ransacDatasetSize(const Eigen::MatrixBase<Derived>& dataset) {
  return dataset.cols();
}

/** A generic RANSAC implementation. By default, the input "dataset" and output
 * "model" are matrices, but this can be changed via template arguments to be
 * any user-defined type. Define ransacDatasetSize() for your custom data types.
 *
 * See \a RANSAC_Template::execute for more info on usage, and examples under
 * `[MRPT]/samples/math_ransac_*`.
 *
 * \sa mrpt::math::ModelSearch, another RANSAC implementation where
 * models can be anything else, not only matrices, and capable of genetic
 * algorithms.
 *
 * \note New in MRPT 2.0.2: The second and third template arguments.
 */
template <typename T>
class RANSAC_Template {
 public:
  RANSAC_Template() = default;

  /** The type of the function passed to mrpt::math::ransac - See the
   * documentation for that method for more info. */
  using TRansacFitFunctor =
      std::function<void(const T& allData, const std::vector<size_t>& useIndices, std::vector<T>& fitModels)>;

  /** The type of the function passed to mrpt::math::ransac  - See the
   * documentation for that method for more info. */
  using TRansacDistanceFunctor =
      std::function<void(const T& allData, const std::vector<T>& testModels, const double distanceThreshold,
                         unsigned int& out_bestModelIndex, std::vector<size_t>& out_inlierIndices)>;

  /** The type of the function passed to mrpt::math::ransac  - See the
   * documentation for that method for more info. */
  using TRansacDegenerateFunctor =
      std::function<bool(const Eigen::MatrixXd& allData, const std::vector<size_t>& useIndices)>;

  /** An implementation of the RANSAC algorithm for robust fitting of models
   * to data.
   *
   *  \param data A DxN matrix with all the observed data. D is the
   * dimensionality of data points and N the number of points.
   *  \param
   *
   *  This implementation is highly inspired on Peter Kovesi's MATLAB scripts
   * (http://www.csse.uwa.edu.au/~pk).
   * \return false if no good solution can be found, true on success.
   * \note [MRPT 1.5.0] `verbose` parameter has been removed, supersedded by
   * COutputLogger settings.
   */
  bool execute(const T& data, const TRansacFitFunctor& fit_func, const TRansacDistanceFunctor& dist_func,
               const TRansacDegenerateFunctor& degen_func, double distanceThreshold,
               unsigned int minimumSizeSamplesToFit, std::vector<size_t>& out_best_inliers, T& out_best_model,
               double prob_good_sample = 0.999, size_t maxIter = 100) const;

};  // end class

/** The default instance of RANSAC, for double type */
using RANSAC = RANSAC_Template<Eigen::MatrixXd>;

/** @} */
}  // namespace ransac
}  // namespace algorithm

#include "ransac_impl.h"
