// Hyperparameters shared by both range-gated detectors: the near-range
// clustering pipeline (city_block.h) and range_gate's far-range
// preprocessing (crop + ground removal ahead of PointPillars). Centralized
// here so "the ROI" and "what counts as ground" mean the same thing on both
// sides of the range gate instead of drifting independently.
#ifndef DETECTION_PARAMS_H_
#define DETECTION_PARAMS_H_

#include <Eigen/Dense>

namespace DetectionParams
{
  inline Eigen::Vector4f roiMin() { return Eigen::Vector4f(-10, -6.5, -2, 1); }
  inline Eigen::Vector4f roiMax() { return Eigen::Vector4f(30, 6.5, 1, 1); }

  constexpr int ransacMaxIterations = 40;
  constexpr float ransacDistanceThreshold = 0.3f;
}

#endif /* DETECTION_PARAMS_H_ */
