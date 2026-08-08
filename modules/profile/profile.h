#ifndef RXS_PROFILE_H
#define RXS_PROFILE_H

/**
 * @file profile.h
 * @brief Point cloud profile measurement module (skeleton)
 *
 * Provides profile evaluation using minimum zone method:
 * - computeProfile():  Compute profile deviation from nominal CAD model
 * - compareWithCAD():  Compare measured cloud against CAD reference
 * - getDeviation():    Get per-point deviation distribution
 *
 * @warning SKELETON ONLY. Original source code lost. See docs/ for RD specs
 *          and legacy DLL SDK reference (computeProfileUsingPSO.dll).
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace rxs {

// Type aliases
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CP;

/**
 * @brief Profile measurement result
 */
struct ProfileResult {
    float profile;              ///< Profile deviation (peak-to-valley)
    float rmse;                 ///< RMS deviation from CAD
    float maxPositiveDev;       ///< Max positive deviation (above CAD)
    float maxNegativeDev;       ///< Max negative deviation (below CAD)
    size_t pointCount;          ///< Number of points evaluated
    bool valid;                 ///< Whether computation succeeded
    std::string error;          ///< Error message if !valid
};

/**
 * @brief Configuration for profile evaluation
 */
struct ProfileConfig {
    float searchRadius = 1.0f;      ///< KNN search radius for CAD projection
    bool usePSO = true;             ///< Use PSO optimization (legacy DLL method)
    int psoIterations = 200;        ///< PSO iteration count
    float psoInertia = 0.7f;        ///< PSO inertia weight
    float convergenceThreshold = 0.001f; ///< Convergence threshold
};

// ---------------------------------------------------------------------------
// Core profile functions
// ---------------------------------------------------------------------------

/**
 * @brief Compute profile deviation from nominal CAD model
 *
 * Evaluates how well the measured point cloud conforms to the nominal
 * CAD surface. Uses minimum zone method or PSO optimization.
 *
 * @param measured   Measured point cloud
 * @param nominal    Nominal CAD reference cloud
 * @param config     Profile evaluation configuration
 * @return ProfileResult with deviation metrics
 *
 * @todo Implement — original source lost. See docs/RD-03002-001 and
 *       docs/最小区域法轮廓度检测算子.docx for algorithm specification.
 */
ProfileResult computeProfile(CP measured, CP nominal, const ProfileConfig& config);

/**
 * @brief Compare measured cloud against CAD reference
 *
 * Computes per-point deviation by projecting measured points onto
 * the CAD surface and computing signed distances.
 *
 * @param measured   Measured point cloud
 * @param nominal    Nominal CAD reference cloud
 * @return Vector of signed deviations (mm)
 *
 * @todo Implement
 */
std::vector<float> compareWithCAD(CP measured, CP nominal);

/**
 * @brief Get per-point deviation distribution
 *
 * @return Vector of deviation values from last computeProfile() call
 *
 * @todo Implement
 */
std::vector<float> getDeviation();

} // namespace rxs

#endif // RXS_PROFILE_H
