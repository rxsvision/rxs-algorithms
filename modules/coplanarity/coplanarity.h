#ifndef RXS_COPLANARITY_H
#define RXS_COPLANARITY_H

/**
 * @file coplanarity.h
 * @brief Point cloud coplanarity measurement module
 *
 * Evaluates coplanarity between reference planes and evaluation planes:
 * - PCA-based reference plane normal estimation
 * - Projection of evaluation planes onto the normal direction
 * - Height deviation between evaluation and reference planes
 *
 * Algorithm:
 * 1. Extract and merge reference plane points from ROI regions
 * 2. PCA fit: smallest eigenvalue direction = reference plane normal
 * 3. Project evaluation plane points onto normal direction
 * 4. Deviation = eval_plane_mean_z - ref_plane_mean_z
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace rxs {

// Type aliases (consistent with flatness module)
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CP;

/**
 * @brief Region of Interest specification (axis-aligned bounding box)
 *
 * Layout: [x_min, x_max, y_min, y_max, z_min, z_max]
 * Supports aggregate initialization: ROIType{xmin, xmax, ymin, ymax, zmin, zmax}
 */
struct ROIType {
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;
};

/**
 * @brief Coplanarity measurement result
 */
struct CoplanarityResult {
    std::vector<float> deviations;  ///< Height deviation of each evaluation plane from reference
    Eigen::Vector3f normal;         ///< Reference plane normal vector (normalized, PCA smallest eigenvalue)
    float refZ;                     ///< Reference plane average height along normal
    bool valid;                     ///< Whether computation succeeded
    std::string error;              ///< Error message if !valid
};

/**
 * @brief Compute coplanarity with full result (normal, refZ, deviations)
 *
 * @param cloud            Input point cloud
 * @param ref_planes       Reference plane ROI specifications
 * @param estimate_planes  Evaluation plane ROI specifications
 * @return CoplanarityResult with deviations and plane metadata
 */
CoplanarityResult coplanarityEx(CP cloud,
                                 const std::vector<ROIType>& ref_planes,
                                 const std::vector<ROIType>& estimate_planes);

/**
 * @brief Compute coplanarity (simple API, backward compatible)
 *
 * @param cloud            Input point cloud
 * @param ref_planes       Reference plane ROI specifications
 * @param estimate_planes  Evaluation plane ROI specifications
 * @return Vector of height deviations (one per evaluation plane)
 */
std::vector<float> coplanarity(CP cloud,
                                const std::vector<ROIType>& ref_planes,
                                const std::vector<ROIType>& estimate_planes);

} // namespace rxs

#endif // RXS_COPLANARITY_H
