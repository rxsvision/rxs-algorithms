/**
 * @file hole_depth.cpp
 * @brief Hole depth measurement algorithm implementation
 *
 * Reconstructed from:
 *   - rxs-toolkit/Hole.cpp (KNNBoundExtract, centerFit)
 *   - rxs-toolkit/Hole.hpp  (findHole, removeRectangle, Hull)
 *   - Original holeDeepth API signature: bool holeDeepth(rxsPointCouldp*, unsigned, rxsResultReport*, double step)
 *
 * Algorithm pipeline:
 *   1. Voxel grid downsampling (optional)
 *   2. Radius outlier removal → detect sparse regions (holes)
 *   3. For each hole:
 *      a. PassThrough filter to isolate local region
 *      b. KNN boundary extraction
 *      c. Surface Z estimation from boundary
 *      d. Bottom Z estimation from center
 *      e. Depth = surfaceZ - bottomZ
 *      f. Radius estimation from boundary extent
 *
 * @author RXS Vision
 * @license BSL 1.1
 */

#include "hole_depth.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

/**
 * @brief Voxel grid downsampling
 */
CP downsample(CP cloud, double voxelSize) {
    CP filtered(new CloudT);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(static_cast<float>(voxelSize),
                     static_cast<float>(voxelSize),
                     static_cast<float>(voxelSize));
    sor.filter(*filtered);
    return filtered;
}

/**
 * @brief Radius outlier removal — extract sparse points (holes)
 *
 * Points with fewer than minNeighbors within searchRadius are
 * classified as outliers (potential hole regions).
 *
 * Adapted from Hole.hpp findHole() and Hole.cpp KNNBoundExtract()
 */
CP removeDensePoints(CP cloud, double searchRadius, int minNeighbors) {
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(static_cast<float>(searchRadius));
    outrem.setMinNeighborsInRadius(minNeighbors);
    outrem.setNegative(true);  // Get the sparse (outlier) points

    pcl::IndicesPtr inliers(new std::vector<int>);
    outrem.filter(*inliers);

    CP filtered(new CloudT);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filtered);
    return filtered;
}

/**
 * @brief Remove rectangular border artifacts
 *
 * If the cloud spans a large rectangular area (>10mm in any direction),
 * trim the edges to remove frame/clamping artifacts.
 *
 * Adapted from Hole.hpp removeRectangle()
 */
void removeRectangle(CP cloud) {
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    float dx = maxPt.x - minPt.x;
    float dy = maxPt.y - minPt.y;

    pcl::PassThrough<PointT> pass;
    while (dy > 10.0f || dx > 10.0f) {
        if (dy > 10.0f) {
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(minPt.x + 1.0f, maxPt.x - 1.0f);
            pass.filter(*cloud);
            pcl::getMinMax3D(*cloud, minPt, maxPt);
            dy = maxPt.y - minPt.y;
            dx = maxPt.x - minPt.x;
        }
        if (dx > 10.0f) {
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(minPt.y + 1.0f, maxPt.y - 1.0f);
            pass.filter(*cloud);
            pcl::getMinMax3D(*cloud, minPt, maxPt);
            dx = maxPt.x - minPt.x;
            dy = maxPt.y - minPt.y;
        }
    }
}

/**
 * @brief Cluster sparse points into separate hole regions
 *
 * Uses Euclidean cluster extraction to group outlier points
 * into individual hole candidates.
 */
std::vector<CP> clusterHoles(CP sparsePoints, double clusterTolerance = 1.0) {
    std::vector<CP> holes;
    if (sparsePoints->empty()) return holes;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(500000);

    std::vector<pcl::PointIndices> clusterIndices;
    ec.setInputCloud(sparsePoints);
    ec.extract(clusterIndices);

    pcl::ExtractIndices<PointT> extract;
    for (const auto& idx : clusterIndices) {
        CP hole(new CloudT);
        extract.setInputCloud(sparsePoints);
        extract.setIndices(boost::make_shared<const pcl::PointIndices>(idx));
        extract.setNegative(false);
        extract.filter(*hole);
        holes.push_back(hole);
    }
    return holes;
}

/**
 * @brief Compute centroid of a point cloud
 */
PointT computeCentroid(CP cloud) {
    Eigen::VectorXf rowMean = cloud->getMatrixXfMap(3, 4, 0).rowwise().mean();
    return PointT(rowMean[0], rowMean[1], rowMean[2]);
}

/**
 * @brief Estimate hole radius from boundary points
 *
 * Computes the mean distance from centroid to all boundary points.
 */
double estimateRadius(CP boundary, const PointT& center) {
    if (boundary->empty()) return 0.0;
    double sumDist = 0.0;
    for (const auto& p : boundary->points) {
        double dx = p.x - center.x;
        double dy = p.y - center.y;
        sumDist += std::sqrt(dx * dx + dy * dy);
    }
    return sumDist / boundary->size();
}

/**
 * @brief KNN-based boundary extraction
 *
 * Classifies points as boundary points based on local neighborhood
 * density and mean displacement. Points with sparse neighborhoods
 * and large mean displacements are classified as boundary.
 *
 * Adapted from Hole.cpp Hole::KNNBoundExtract() and Hole.hpp KNNBoundExtract()
 */
CP knnBoundaryExtract(CP cloud, double radius, int maxNeighbors, double diffThreshold) {
    CP boundary(new CloudT);
    if (cloud->empty()) return boundary;

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i) {
        PointT searchPoint = cloud->points[i];
        std::vector<int> idxSearch;
        std::vector<float> distSquared;

        if (kdtree.radiusSearch(searchPoint, radius, idxSearch, distSquared) > 0) {
            int K = static_cast<int>(idxSearch.size());
            if (K > maxNeighbors) continue;

            float sumX = 0.0f, sumY = 0.0f;
            for (int j = 0; j < K; ++j) {
                int nIdx = idxSearch[j];
                sumX += cloud->points[nIdx].x;
                sumY += cloud->points[nIdx].y;
            }

            float meanX = std::abs(sumX / K - searchPoint.x);
            float meanY = std::abs(sumY / K - searchPoint.y);
            float diff = std::max(meanX, meanY);

            if (diff > diffThreshold) {
                boundary->push_back(searchPoint);
            }
        }
    }
    return boundary;
}

/**
 * @brief Estimate surface Z from boundary points
 *
 * The boundary points represent the rim of the hole, which sits
 * on the surrounding surface. Use median Z for robustness.
 */
double estimateSurfaceZ(CP boundary) {
    if (boundary->empty()) return 0.0;
    std::vector<double> zVals;
    zVals.reserve(boundary->size());
    for (const auto& p : boundary->points) {
        zVals.push_back(p.z);
    }
    std::sort(zVals.begin(), zVals.end());
    return zVals[zVals.size() / 2];  // Median
}

/**
 * @brief Estimate bottom Z from hole interior points
 *
 * Use a percentile-based approach to avoid outlier influence.
 */
double estimateBottomZ(CP holeCloud, double percentile = 0.1) {
    if (holeCloud->empty()) return 0.0;
    std::vector<double> zVals;
    zVals.reserve(holeCloud->size());
    for (const auto& p : holeCloud->points) {
        zVals.push_back(p.z);
    }
    std::sort(zVals.begin(), zVals.end());
    size_t idx = static_cast<size_t>(zVals.size() * percentile);
    if (idx >= zVals.size()) idx = zVals.size() - 1;
    return zVals[idx];
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Public API implementation
// ---------------------------------------------------------------------------

HOLE_DEPTH_API std::vector<HoleDepthResult> measureHoleDepth(
    CP cloud,
    const HoleDepthConfig& config)
{
    std::vector<HoleDepthResult> results;
    if (!cloud || cloud->empty()) return results;

    // Step 1: Optional voxel grid downsampling
    CP processed;
    if (config.enableVoxelGrid && config.voxelSize > 0) {
        processed = downsample(cloud, config.voxelSize);
    } else {
        processed.reset(new CloudT(*cloud));
    }

    // Step 2: Remove border artifacts
    removeRectangle(processed);

    // Step 3: Find sparse regions (holes) via outlier removal
    CP sparsePoints = removeDensePoints(
        processed, config.searchRadius, config.minNeighbors);

    if (sparsePoints->empty()) return results;

    // Step 4: Cluster sparse points into individual holes
    std::vector<CP> holes = clusterHoles(sparsePoints, config.searchRadius);

    // Step 5: For each hole, measure depth
    for (CP holeCloud : holes) {
        if (holeCloud->size() < 20) continue;

        HoleDepthResult result;
        result.valid = false;

        // Compute hole center
        PointT center = computeCentroid(holeCloud);
        result.centerX = center.x;
        result.centerY = center.y;
        result.centerZ = center.z;

        // Extract local region around center for boundary analysis
        CP localRegion(new CloudT);
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(processed);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(
            static_cast<float>(center.x - config.passFilterWidth),
            static_cast<float>(center.x + config.passFilterWidth));
        pass.filter(*localRegion);
        pass.setInputCloud(localRegion);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(
            static_cast<float>(center.y - config.passFilterWidth),
            static_cast<float>(center.y + config.passFilterWidth));
        pass.filter(*localRegion);

        if (localRegion->empty()) continue;

        // Extract boundary using KNN
        CP boundary = knnBoundaryExtract(
            localRegion,
            config.knnRadius,
            config.knnThreshold,
            config.knnDiffThreshold);

        if (boundary->size() < 5) {
            // Fallback: use all local region points as boundary
            boundary = localRegion;
        }

        // Estimate surface Z from boundary
        result.surfaceZ = estimateSurfaceZ(boundary);

        // Estimate bottom Z from hole interior
        result.centerZ = estimateBottomZ(holeCloud, 0.1);

        // Depth = surface - bottom (positive if hole goes down)
        result.depth = result.surfaceZ - result.centerZ;

        // Estimate radius
        result.radius = estimateRadius(boundary, center);

        result.valid = true;
        results.push_back(result);
    }

    return results;
}

HOLE_DEPTH_API std::vector<HoleDepthResult> measureHoleDepth(
    CP cloud,
    double step)
{
    HoleDepthConfig config;
    config.step = step;
    return measureHoleDepth(cloud, config);
}

HOLE_DEPTH_API std::vector<PointT> findHoleCenters(
    CP cloud,
    double searchRadius,
    int minNeighbors)
{
    std::vector<PointT> centers;
    if (!cloud || cloud->empty()) return centers;

    CP sparsePoints = removeDensePoints(cloud, searchRadius, minNeighbors);
    if (sparsePoints->empty()) return centers;

    std::vector<CP> holes = clusterHoles(sparsePoints, searchRadius);
    for (CP hole : holes) {
        if (hole->size() < 20) continue;
        centers.push_back(computeCentroid(hole));
    }
    return centers;
}

HOLE_DEPTH_API CP extractHoleBoundary(
    CP cloud,
    double radius,
    int maxNeighbors,
    double diffThreshold)
{
    return knnBoundaryExtract(cloud, radius, maxNeighbors, diffThreshold);
}

HOLE_DEPTH_API HoleDepthResult measureDepthAtLocation(
    CP cloud,
    const PointT& center,
    double width,
    double step)
{
    HoleDepthResult result;
    result.valid = false;
    result.centerX = center.x;
    result.centerY = center.y;
    result.centerZ = center.z;
    result.depth = 0;
    result.radius = 0;
    result.surfaceZ = 0;

    if (!cloud || cloud->empty()) return result;

    // Extract local region
    CP localRegion(new CloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(
        static_cast<float>(center.x - width),
        static_cast<float>(center.x + width));
    pass.filter(*localRegion);
    pass.setInputCloud(localRegion);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(
        static_cast<float>(center.y - width),
        static_cast<float>(center.y + width));
    pass.filter(*localRegion);

    if (localRegion->empty()) return result;

    // Extract boundary
    CP boundary = knnBoundaryExtract(localRegion, 1.0, 15, 0.05);
    if (boundary->size() < 5) boundary = localRegion;

    // Measure depth
    result.surfaceZ = estimateSurfaceZ(boundary);

    // For bottom Z, use points near center
    CP centerRegion(new CloudT);
    double innerWidth = width * 0.3;
    pass.setInputCloud(localRegion);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(
        static_cast<float>(center.x - innerWidth),
        static_cast<float>(center.x + innerWidth));
    pass.filter(*centerRegion);
    pass.setInputCloud(centerRegion);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(
        static_cast<float>(center.y - innerWidth),
        static_cast<float>(center.y + innerWidth));
    pass.filter(*centerRegion);

    if (!centerRegion->empty()) {
        result.centerZ = estimateBottomZ(centerRegion, 0.1);
    }

    result.depth = result.surfaceZ - result.centerZ;
    result.radius = estimateRadius(boundary, center);
    result.valid = true;
    return result;
}

HOLE_DEPTH_API std::string formatResults(
    const std::vector<HoleDepthResult>& results)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(5);
    ss << "Hole Depth Measurement Results\n";
    ss << "=" << std::string(60, '=') << "\n";
    ss << std::left
       << std::setw(5)  << "ID"
       << std::setw(12) << "CenterX"
       << std::setw(12) << "CenterY"
       << std::setw(12) << "BottomZ"
       << std::setw(12) << "SurfaceZ"
       << std::setw(10) << "Depth"
       << std::setw(10) << "Radius"
       << "Valid\n";
    ss << "-" << std::string(70, '-') << "\n";

    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        ss << std::left
           << std::setw(5)  << (i + 1)
           << std::setw(12) << r.centerX
           << std::setw(12) << r.centerY
           << std::setw(12) << r.centerZ
           << std::setw(12) << r.surfaceZ
           << std::setw(10) << r.depth
           << std::setw(10) << r.radius
           << (r.valid ? "OK" : "FAIL")
           << "\n";
    }
    ss << "=" << std::string(60, '=') << "\n";
    ss << "Total holes detected: " << results.size() << "\n";
    return ss.str();
}
