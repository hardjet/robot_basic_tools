/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include "algorithm/random/RandomGenerators.h"

#include <cstddef>
#include <vector>

// To be included from ransac.h only

namespace algorithm {
namespace ransac {
template <typename T>
bool RANSAC_Template<T>::execute(const T& data, const TRansacFitFunctor& fit_func,
                                 const TRansacDistanceFunctor& dist_func, const TRansacDegenerateFunctor& degen_func,
                                 const double distanceThreshold, const unsigned int minimumSizeSamplesToFit,
                                 std::vector<size_t>& out_best_inliers, T& out_best_model, const double p,
                                 const size_t maxIter) const {
  // Highly inspired on http://www.csse.uwa.edu.au/~pk/

  assert(minimumSizeSamplesToFit > 1u);

  const size_t Npts = ransacDatasetSize(data);

  assert(Npts > 1);

  // Maximum number of attempts to select a non-degenerate data set.
  const size_t maxDataTrials = 100;

  // Sentinel value allowing detection of solution failure.
  out_best_model = T();
  out_best_inliers.clear();

  size_t trialcount = 0;
  size_t bestscore = std::string::npos;  // npos will mean "none"
  size_t N = 1;                          // Dummy initialisation for number of trials.

  std::vector<size_t> ind(minimumSizeSamplesToFit);

  while (N > trialcount) {
    // Select at random s datapoints to form a trial model, M.
    // In selecting these points we have to check that they are not in
    // a degenerate configuration.
    bool degenerate = true;
    size_t count = 1;
    std::vector<T> MODELS;

    while (degenerate) {
      // Generate s random indicies in the range 1..npts
      ind.resize(minimumSizeSamplesToFit);

      // The +0.99... is due to the floor rounding afterwards when
      // converting from random double samples to size_t
      algorithm::random::getRandomGenerator().drawUniformVector(ind, 0.0, Npts - 1 + 0.999999);

      // printf("ind: %d-%d\n", ind[0], ind[1]);
      // Test that these points are not a degenerate configuration.
      degenerate = degen_func(data, ind);

      if (!degenerate) {
        // Fit model to this random selection of data points.
        // Note that M may represent a set of models that fit the data
        fit_func(data, ind, MODELS);

        // Depending on your problem it might be that the only way you
        // can determine whether a data set is degenerate or not is to
        // try to fit a model and see if it succeeds.  If it fails we
        // reset degenerate to true.
        degenerate = MODELS.empty();
      }

      // Safeguard against being stuck in this loop forever
      if (++count > maxDataTrials) {
        printf("Unable to select a nondegenerate data set\n");
        break;
      }
    }

    // Once we are out here we should have some kind of model...
    // Evaluate distances between points and model returning the indices
    // of elements in x that are inliers.  Additionally, if M is a cell
    // array of possible models 'distfn' will return the model that has
    // the most inliers.  After this call M will be a non-cell objec
    // representing only one model.
    unsigned int bestModelIdx = std::numeric_limits<unsigned int>::max();
    std::vector<size_t> inliers;
    if (!degenerate) {
      dist_func(data, MODELS, double(distanceThreshold), bestModelIdx, inliers);
      assert(bestModelIdx < MODELS.size());
    }

    // Find the number of inliers to this model.
    const size_t ninliers = inliers.size();
    // Always update on the first iteration, regardless of the result (even
    // for ninliers=0)
    bool update_estim_num_iters = (trialcount == 0);

    if (ninliers > bestscore || (bestscore == std::string::npos && ninliers != 0)) {
      bestscore = ninliers;  // Record data for this model

      out_best_model = std::move(MODELS[bestModelIdx]);
      out_best_inliers = std::move(inliers);
      update_estim_num_iters = true;
    }

    if (update_estim_num_iters) {
      // Update estimate of N, the number of trials to ensure we pick,
      // with probability p, a data set with no outliers.
      double fracinliers = ninliers / static_cast<double>(Npts);
      double pNoOutliers = 1 - pow(fracinliers, static_cast<double>(minimumSizeSamplesToFit));

      pNoOutliers = std::max(std::numeric_limits<double>::epsilon(),
                             pNoOutliers);  // Avoid division by -Inf
      pNoOutliers = std::min(1.0 - std::numeric_limits<double>::epsilon(),
                             pNoOutliers);  // Avoid division by 0.
      // Number of
      N = static_cast<size_t>(log(1 - p) / log(pNoOutliers));
      // printf("Iter #%u Estimated number of iters: %u  pNoOutliers = %f #inliers: %u\n", (unsigned)trialcount,
      //        (unsigned)N, pNoOutliers, (unsigned)ninliers);
    }

    ++trialcount;

    // printf("trial %u out of %u\n", (unsigned int)trialcount, (unsigned int)ceil(static_cast<double>(N)));

    // Safeguard against being stuck in this loop forever
    if (trialcount > maxIter) {
      printf("Warning: maximum number of trials (%u) reached\n", (unsigned)maxIter);
      break;
    }
  }

  if (!out_best_inliers.empty()) {  // We got a solution
    // printf("Finished in %u iterations.\n", (unsigned)trialcount);
    return true;
  } else {
    printf("Finished without any proper solution\n");
    return false;
  }
}
}  // namespace ransac
}  // namespace algorithm
