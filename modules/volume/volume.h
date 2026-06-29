#ifndef RXS_VOLUME_H
#define RXS_VOLUME_H

/**
 * @file volume.h
 * @brief Point cloud volume calculation module
 *
 * Provides multiple volume computation strategies:
 * - volume():           Original Riemann sum (backward compatible)
 * - volumeRect():       Rectilinear grid with separate dx/dy spacing
 * - volumeAbovePlane(): Volume above a reference Z plane (glue/height measurement)
 * - volumeConvexHull(): Convex hull based volume (irregular boundaries)
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <vector>

namespace rxs {

// Type aliases (match existing codebase conventions)
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CP;

/**
 * @brief Volume computation result
 */
struct VolumeResult {
    double volume;       ///< Computed volume (cubic units)
    size_t pointCount;   ///< Number of points used
    bool valid;          ///< Whether computation succeeded
    std::string error;   ///< Error message if !valid
};

/**
 * @brief Compute volume via Riemann sum (original API, backward compatible)
 *
 * V = sum(xyInterval^2 * z_i) for each point i
 *
 * Assumes uniform square grid spacing and all z >= 0.
 * For height-from-plane measurement, use volumeAbovePlane() instead.
 *
 * @param cloud       Input point cloud
 * @param xyInterval  Uniform XY grid spacing
 * @return Volume value (0 if cloud is null/empty)
 */
double volume(CP cloud, double xyInterval);

/**
 * @brief Compute volume via Riemann sum with separate X/Y spacing
 *
 * V = sum(dx * dy * z_i) for each point i
 *
 * @param cloud  Input point cloud
 * @param dx     X-direction grid spacing
 * @param dy     Y-direction grid spacing
 * @return VolumeResult with computed volume and metadata
 */
VolumeResult volumeRect(CP cloud, double dx, double dy);

/**
 * @brief Compute volume above a reference Z plane
 *
 * V = sum(dx * dy * max(0, z_i - planeZ)) for each point i
 *
 * Used for glue volume, material height measurement, etc.
 * Points below the plane are clamped to zero (no negative contribution).
 *
 * @param cloud   Input point cloud
 * @param dx      X-direction grid spacing
 * @param dy      Y-direction grid spacing
 * @param planeZ  Reference plane Z height
 * @return VolumeResult with computed volume and metadata
 */
VolumeResult volumeAbovePlane(CP cloud, double dx, double dy, double planeZ);

/**
 * @brief Compute volume using convex hull method
 *
 * Builds a convex hull from the point cloud and computes the volume
 * between the hull and the XY plane. Suitable for irregular boundaries
 * where grid spacing is non-uniform or unknown.
 *
 * @param cloud  Input point cloud
 * @return VolumeResult with computed volume and metadata
 */
VolumeResult volumeConvexHull(CP cloud);

/**
 * @brief Compute volume using convex hull method with reference plane
 *
 * Similar to volumeConvexHull() but subtracts planeZ from all Z values
 * before computing the hull volume.
 *
 * @param cloud   Input point cloud
 * @param planeZ  Reference plane Z height
 * @return VolumeResult with computed volume and metadata
 */
VolumeResult volumeConvexHullAbovePlane(CP cloud, double planeZ);

} // namespace rxs

#endif // RXS_VOLUME_H
