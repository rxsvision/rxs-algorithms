#include "volume.h"

#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace rxs {

// ---------------------------------------------------------------------------
// Original API (backward compatible)
// ---------------------------------------------------------------------------

double volume(CP cloud, double xyInterval)
{
    if (!cloud || cloud->empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (const auto& p : cloud->points) {
        // Skip NaN/Inf points
        if (!std::isfinite(p.z)) continue;
        sum += xyInterval * xyInterval * p.z;
    }
    return sum;
}

// ---------------------------------------------------------------------------
// Enhanced APIs
// ---------------------------------------------------------------------------

VolumeResult volumeRect(CP cloud, double dx, double dy)
{
    VolumeResult result = {0.0, 0, false, ""};

    if (!cloud) {
        result.error = "Null cloud pointer";
        return result;
    }
    if (cloud->empty()) {
        result.error = "Empty cloud";
        return result;
    }
    if (dx <= 0 || dy <= 0) {
        result.error = "Invalid grid spacing (dx/dy must be > 0)";
        return result;
    }

    double sum = 0.0;
    size_t count = 0;
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.z)) continue;
        sum += dx * dy * p.z;
        count++;
    }

    result.volume = sum;
    result.pointCount = count;
    result.valid = true;
    return result;
}

VolumeResult volumeAbovePlane(CP cloud, double dx, double dy, double planeZ)
{
    VolumeResult result = {0.0, 0, false, ""};

    if (!cloud) {
        result.error = "Null cloud pointer";
        return result;
    }
    if (cloud->empty()) {
        result.error = "Empty cloud";
        return result;
    }
    if (dx <= 0 || dy <= 0) {
        result.error = "Invalid grid spacing (dx/dy must be > 0)";
        return result;
    }

    double sum = 0.0;
    size_t count = 0;
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.z)) continue;
        double h = p.z - planeZ;
        if (h > 0.0) {
            sum += dx * dy * h;
        }
        count++;
    }

    result.volume = sum;
    result.pointCount = count;
    result.valid = true;
    return result;
}

VolumeResult volumeConvexHull(CP cloud)
{
    VolumeResult result = {0.0, 0, false, ""};

    if (!cloud) {
        result.error = "Null cloud pointer";
        return result;
    }
    if (cloud->empty()) {
        result.error = "Empty cloud";
        return result;
    }
    if (cloud->size() < 4) {
        result.error = "Need at least 4 points for convex hull";
        return result;
    }

    // Compute convex hull
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);

    pcl::PolygonMesh mesh;
    hull.reconstruct(mesh);

    // Compute volume from mesh using signed tetrahedron method
    double vol = 0.0;
    auto& vertices = mesh.cloud.data;
    auto& polygons = mesh.polygons;

    // Get vertex coordinates as Eigen matrix
    pcl::PointCloud<PointT>::Ptr hullCloud(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(mesh.cloud, *hullCloud);

    for (const auto& poly : polygons) {
        if (poly.vertices.size() < 3) continue;

        // Use first vertex as origin, compute signed volume of tetrahedra
        Eigen::Vector3f v0 = hullCloud->points[poly.vertices[0]].getVector3fMap();

        for (size_t i = 1; i < poly.vertices.size() - 1; ++i) {
            Eigen::Vector3f v1 = hullCloud->points[poly.vertices[i]].getVector3fMap();
            Eigen::Vector3f v2 = hullCloud->points[poly.vertices[i + 1]].getVector3fMap();
            // Signed volume of tetrahedron (origin, v0, v1, v2)
            vol += std::abs((v1 - v0).cross(v2 - v0).dot(v0)) / 6.0;
        }
    }

    result.volume = vol;
    result.pointCount = cloud->size();
    result.valid = true;
    return result;
}

VolumeResult volumeConvexHullAbovePlane(CP cloud, double planeZ)
{
    VolumeResult result = {0.0, 0, false, ""};

    if (!cloud) {
        result.error = "Null cloud pointer";
        return result;
    }
    if (cloud->empty()) {
        result.error = "Empty cloud";
        return result;
    }

    // Shift all points so planeZ becomes 0
    CP shifted(new CloudT);
    *shifted = *cloud;
    for (auto& p : shifted->points) {
        p.z -= planeZ;
        // Clamp negative values to 0 (points below plane don't contribute)
        if (p.z < 0.0f) p.z = 0.0f;
    }

    return volumeConvexHull(shifted);
}

} // namespace rxs
