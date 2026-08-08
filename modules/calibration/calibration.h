#ifndef RXS_CALIBRATION_H
#define RXS_CALIBRATION_H

/**
 * @file calibration.h
 * @brief Point cloud calibration module (skeleton)
 *
 * Provides two-cloud calibration:
 * - calibrate():      Compute rigid transform between source and target clouds
 * - computeError():   Evaluate calibration RMS error
 * - getTransform():   Retrieve the computed 4x4 transformation
 *
 * @warning SKELETON ONLY. Original source code lost. See docs/ for RD specs,
 *          pseudocode, and legacy DLL SDK reference.
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <string>

namespace rxs {

// Type aliases
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CP;

/**
 * @brief Calibration result
 */
struct CalibrationResult {
    Eigen::Matrix4f transform;   ///< 4x4 rigid transformation matrix
    float rmse;                  ///< Root mean square registration error
    size_t pointCount;           ///< Number of correspondences used
    bool valid;                  ///< Whether calibration succeeded
    std::string error;           ///< Error message if !valid
};

/**
 * @brief Configuration for calibration
 */
struct CalibrationConfig {
    float distanceThreshold = 1.0f;  ///< Max correspondence distance
    int maxIterations = 100;         ///< Max ICP iterations
    float epsilon = 1e-6f;          ///< Convergence threshold
    bool useRANSAC = true;           ///< Pre-align with RANSAC before ICP
};

// ---------------------------------------------------------------------------
// Core calibration functions
// ---------------------------------------------------------------------------

/**
 * @brief Calibrate two point clouds (source -> target)
 *
 * Computes the rigid transformation that aligns source to target.
 * Pipeline: RANSAC coarse alignment -> ICP fine registration.
 *
 * @param source   Source point cloud
 * @param target   Target point cloud
 * @param config   Calibration configuration
 * @return CalibrationResult with transform and error metrics
 *
 * @todo Implement — original source lost. See docs/RD-03002-001 and
 *       docs/标定算法伪代码.pdf for algorithm specification.
 */
CalibrationResult calibrate(CP source, CP target, const CalibrationConfig& config);

/**
 * @brief Compute registration error between two clouds
 *
 * @param source   Source point cloud
 * @param target   Target point cloud
 * @param transform  Transformation to apply to source
 * @return RMS error
 *
 * @todo Implement
 */
float computeError(CP source, CP target, const Eigen::Matrix4f& transform);

/**
 * @brief Get the last computed transformation
 *
 * @return 4x4 transformation matrix
 *
 * @todo Implement
 */
Eigen::Matrix4f getTransform();

} // namespace rxs

#endif // RXS_CALIBRATION_H
