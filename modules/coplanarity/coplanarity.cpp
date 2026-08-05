#include "coplanarity.h"

#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <pcl/common/impl/common.hpp>
#include <algorithm>

namespace rxs {

// ---------------------------------------------------------------------------
// Internal helper: extract axis-aligned ROI region from point cloud
// Replaces pclProcess::getROI() dependency from legacy czxTool.h
// ---------------------------------------------------------------------------
static CP extractROI(CP cloud, const ROIType& roi)
{
    CP filtered(new CloudT);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);

    // X axis
    pass.setFilterFieldName("x");
    pass.setFilterLimits(roi.x_min, roi.x_max);
    pass.filter(*filtered);

    // Y axis (chain on previous result)
    pass.setInputCloud(filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(roi.y_min, roi.y_max);
    pass.filter(*filtered);

    // Z axis (chain on previous result)
    pass.setInputCloud(filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(roi.z_min, roi.z_max);
    pass.filter(*filtered);

    return filtered;
}

// ---------------------------------------------------------------------------
// coplanarityEx: full result API
// ---------------------------------------------------------------------------
CoplanarityResult coplanarityEx(CP cloud,
                                 const std::vector<ROIType>& ref_planes,
                                 const std::vector<ROIType>& estimate_planes)
{
    CoplanarityResult result;
    result.valid = false;
    result.refZ = 0.0f;

    if (!cloud || cloud->empty()) {
        result.error = "Input cloud is empty";
        return result;
    }
    if (ref_planes.empty()) {
        result.error = "No reference planes specified";
        return result;
    }

    // 1. Extract and merge reference plane points
    CP ref_plane(new CloudT);
    for (const auto& roi : ref_planes) {
        CP plane = extractROI(cloud, roi);
        *ref_plane += *plane;
    }

    if (ref_plane->empty()) {
        result.error = "Reference plane extraction yielded no points";
        return result;
    }

    // 2. PCA fit: smallest eigenvalue direction = plane normal
    pcl::PCA<PointT> pca;
    pca.setInputCloud(ref_plane);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    result.normal = eigen_vectors.col(2);

    // 3. Reference plane average height along normal direction
    result.refZ = static_cast<float>(
        (result.normal.transpose() * ref_plane->getMatrixXfMap(3, 4, 0)).mean());

    // 4. Evaluation plane deviations
    result.deviations.reserve(estimate_planes.size());
    for (const auto& roi : estimate_planes) {
        CP plane = extractROI(cloud, roi);
        if (plane->empty()) {
            result.deviations.push_back(0.0f);
            continue;
        }
        Eigen::VectorXf z = result.normal.transpose() * plane->getMatrixXfMap(3, 4, 0);
        result.deviations.push_back(static_cast<float>(z.mean() - result.refZ));
    }

    result.valid = true;
    return result;
}

// ---------------------------------------------------------------------------
// coplanarity: simple API (backward compatible)
// ---------------------------------------------------------------------------
std::vector<float> coplanarity(CP cloud,
                                const std::vector<ROIType>& ref_planes,
                                const std::vector<ROIType>& estimate_planes)
{
    auto result = coplanarityEx(cloud, ref_planes, estimate_planes);
    return result.deviations;
}

} // namespace rxs