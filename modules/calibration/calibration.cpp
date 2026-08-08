/**
 * @file calibration.cpp
 * @brief Point cloud calibration module (skeleton implementation)
 *
 * @warning SKELETON ONLY. Original source code lost.
 *          All functions throw std::runtime_error.
 *          See docs/ for RD specs, pseudocode, and legacy DLL SDK reference.
 */

#include "calibration.h"
#include <stdexcept>

namespace rxs {

CalibrationResult calibrate(CP source, CP target, const CalibrationConfig& config) {
    (void)source; (void)target; (void)config;
    throw std::runtime_error(
        "rxs::calibrate() not implemented — original source lost. "
        "See docs/RD-03002-001 and docs/标定算法伪代码.pdf for spec.");
}

float computeError(CP source, CP target, const Eigen::Matrix4f& transform) {
    (void)source; (void)target; (void)transform;
    throw std::runtime_error(
        "rxs::computeError() not implemented — original source lost.");
}

Eigen::Matrix4f getTransform() {
    throw std::runtime_error(
        "rxs::getTransform() not implemented — original source lost.");
}

} // namespace rxs
