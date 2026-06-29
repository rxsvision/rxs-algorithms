#ifndef RXS_FLATNESS_H
#define RXS_FLATNESS_H

/**
 * @file flatness.h
 * @brief Point cloud flatness measurement module
 *
 * Provides plane fitting and flatness evaluation:
 * - flatness():           LSM plane fit + peak-to-valley (backward compatible)
 * - flatnessRMSE():       LSM plane fit + RMSE deviation
 * - flatnessMinimumZone(): Convex hull + exhaustive search (MZ method)
 * - extractPlane():       Plane region extraction from noisy cloud
 * - extractCircle():      Circular plane extraction via RANSAC
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
 * @brief Flatness measurement result
 */
struct FlatnessResult {
    float flatness;          ///< Peak-to-valley flatness value
    float rmse;              ///< Root mean square deviation from fitted plane
    PointT minPoint;         ///< Lowest point (max deviation below plane)
    PointT maxPoint;         ///< Highest point (max deviation above plane)
    Eigen::Vector3f normal;  ///< Fitted plane normal vector (normalized)
    float planeD;            ///< Plane equation: normal.dot(p) + planeD = 0
    size_t pointCount;       ///< Number of points used
    bool valid;              ///< Whether computation succeeded
    std::string error;       ///< Error message if !valid
};

/**
 * @brief Configuration for plane extraction
 */
struct FlatnessConfig {
    float zCenter = 0.0f;       ///< Z-axis center for pass-through filter
    float zTolerance = 2.0f;    ///< Z-axis filter half-range
    float clusterTolerance = 0.2f; ///< Euclidean clustering tolerance
    int knnRadius = 5;          ///< KNN boundary search radius multiplier
    float resolution = 0.1f;    ///< Point cloud resolution (for KNN)
    float boundaryTolerance = 0.5f; ///< Boundary extraction tolerance
    float circleRefOffset = 0.4f;   ///< Circle reference distance offset
    float circleTolerance = 0.2f;   ///< Circle fitting tolerance

    // Convenience: set zCenter and zTolerance from a config file
    static FlatnessConfig fromProfile(const std::string& path);
};

// ---------------------------------------------------------------------------
// Core flatness functions
// ---------------------------------------------------------------------------

/**
 * @brief Compute flatness via least-squares plane fit + peak-to-valley
 *
 * Fits a plane using SVD least squares, projects all points onto the
 * plane normal, returns max - min deviation.
 *
 * Original API preserved for backward compatibility.
 *
 * @param cloud      Input plane point cloud
 * @param minPoint   Output: lowest point
 * @param maxPoint   Output: highest point
 * @return Flatness value (max - min deviation from fitted plane)
 */
float flatness(CP cloud, PointT& minPoint, PointT& maxPoint);

/**
 * @brief Compute flatness with full result (RMSE, normal, etc.)
 *
 * @param cloud  Input plane point cloud
 * @return FlatnessResult with all metrics
 */
FlatnessResult flatnessEx(CP cloud);

/**
 * @brief Compute RMSE flatness
 *
 * Fits a plane and returns the root mean square deviation of all points
 * from the fitted plane. More robust to outliers than peak-to-valley.
 *
 * @param cloud  Input plane point cloud
 * @return RMSE value
 */
float flatnessRMSE(CP cloud);

/**
 * @brief Compute minimum zone flatness (convex hull method)
 *
 * Builds a convex hull from the point cloud, then exhaustively searches
 * all triplets of hull points to find the plane that minimizes the
 * peak-to-valley flatness. This is the ISO 1101 definition of flatness.
 *
 * Time complexity: O(n_hull^3) where n_hull is convex hull size.
 * For large clouds, use flatness() or flatnessRMSE() instead.
 *
 * @param cloud  Input plane point cloud
 * @return Minimum zone flatness value
 */
float flatnessMinimumZone(CP cloud);

// ---------------------------------------------------------------------------
// Plane extraction functions
// ---------------------------------------------------------------------------

/**
 * @brief Extract the largest planar region from a noisy point cloud
 *
 * Pipeline: Z pass-through filter -> Euclidean clustering (largest cluster)
 *           -> Statistical distance filtering -> KNN boundary removal
 *
 * @param cloud  Input point cloud (modified in place for filtering)
 * @param config Extraction configuration
 * @return Extracted plane point cloud
 */
CP extractPlane(CP cloud, const FlatnessConfig& config);

/**
 * @brief Extract a circular planar region via RANSAC circle fitting
 *
 * Pipeline: Z pass-through filter -> Euclidean clustering -> KNN boundary
 *           -> RANSAC circle 3D fit -> Ring extraction
 *
 * @param cloud  Input point cloud (modified in place for filtering)
 * @param config Extraction configuration
 * @return Extracted circular ring point cloud
 */
CP extractCircle(CP cloud, const FlatnessConfig& config);

/**
 * @brief Extract the largest cluster from a point cloud
 *
 * Uses Euclidean clustering with configurable tolerance.
 *
 * @param cloud           Input point cloud
 * @param tolerance       Cluster distance tolerance
 * @param minClusterSize  Minimum cluster size (0 = no limit)
 * @param maxClusterSize  Maximum cluster size (0 = no limit)
 * @return Largest cluster point cloud
 */
CP extractLargestCluster(CP cloud, float tolerance,
                         int minClusterSize = 0, int maxClusterSize = 0);

} // namespace rxs

#endif // RXS_FLATNESS_H
