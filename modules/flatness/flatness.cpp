#include "flatness.h"

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <unordered_map>

namespace rxs {

// ---------------------------------------------------------------------------
// FlatnessConfig
// ---------------------------------------------------------------------------

FlatnessConfig FlatnessConfig::fromProfile(const std::string& path)
{
    FlatnessConfig cfg;
    std::ifstream file(path);
    if (!file.is_open()) return cfg;

    std::string line;
    std::unordered_map<std::string, std::string> map;
    while (std::getline(file, line)) {
        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            map[line.substr(0, pos)] = line.substr(pos + 1);
        }
    }

    if (map.count("z")) cfg.zCenter = std::stof(map["z"]);
    if (map.count("resolution")) cfg.resolution = std::stof(map["resolution"]);

    return cfg;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/**
 * @brief Fit a plane using SVD least squares
 * @return Plane coefficients [a, b, d] for z = a*x + b*y + d
 */
static Eigen::Vector3f planeFitLSM(CP cloud)
{
    Eigen::MatrixXf xyz = cloud->getMatrixXfMap(3, 4, 0);
    Eigen::VectorXf z = xyz.row(2);
    xyz.row(2).setOnes();
    Eigen::Vector3f coeff =
        Eigen::JacobiSVD<Eigen::MatrixXf>(
            xyz.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV)
            .solve(z);
    return coeff;
}

/**
 * @brief KNN boundary extraction (points with sparse neighbors)
 * Uses PCL KdTreeFLANN (2D XY projection)
 */
static CP knnBoundary(CP cloud, float radius, int minK)
{
    if (!cloud || cloud->empty()) return cloud;

    CP filtered(new CloudT);

    // Build 2D kd-tree using PCL (project to XY)
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices;
    std::vector<float> dists;

    for (size_t i = 0; i < cloud->size(); ++i) {
        // Use XY distance by setting z=0 for query
        PointT query = cloud->points[i];
        query.z = 0.0f;

        // Search with radius (squared distance)
        int count = kdtree.radiusSearch(query, radius, indices, dists);

        if (count < minK) {
            filtered->push_back(cloud->points[i]);
        }
    }
    return filtered;
}

/**
 * @brief KNN boundary removal (keep points with enough neighbors)
 */
static CP knnBoundaryRemove(CP cloud, float radius, int minK)
{
    if (!cloud || cloud->empty()) return cloud;

    CP filtered(new CloudT);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices;
    std::vector<float> dists;

    for (size_t i = 0; i < cloud->size(); ++i) {
        PointT query = cloud->points[i];
        query.z = 0.0f;

        int count = kdtree.radiusSearch(query, radius, indices, dists);

        if (count > minK) {
            filtered->push_back(cloud->points[i]);
        }
    }
    return filtered;
}

// ---------------------------------------------------------------------------
// Core flatness functions
// ---------------------------------------------------------------------------

float flatness(CP cloud, PointT& minPoint, PointT& maxPoint)
{
    if (!cloud || cloud->empty()) {
        return 0.0f;
    }

    auto coeff = planeFitLSM(cloud);
    Eigen::Vector3f normal = Eigen::Vector3f(coeff[0], coeff[1], -1).normalized();
    Eigen::VectorXf z = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);

    int maxIdx = 0, minIdx = 0;
    float maxVal = z.maxCoeff(&maxIdx);
    float minVal = z.minCoeff(&minIdx);

    minPoint = cloud->points[minIdx];
    maxPoint = cloud->points[maxIdx];

    return maxVal - minVal;
}

FlatnessResult flatnessEx(CP cloud)
{
    FlatnessResult result;
    result.valid = false;
    result.flatness = 0.0f;
    result.rmse = 0.0f;
    result.pointCount = 0;

    if (!cloud) {
        result.error = "Null cloud pointer";
        return result;
    }
    if (cloud->empty()) {
        result.error = "Empty cloud";
        return result;
    }

    // Fit plane
    auto coeff = planeFitLSM(cloud);
    result.normal = Eigen::Vector3f(coeff[0], coeff[1], -1).normalized();
    result.planeD = coeff[2];

    // Compute deviations
    Eigen::VectorXf z = result.normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);

    int maxIdx = 0, minIdx = 0;
    float maxVal = z.maxCoeff(&maxIdx);
    float minVal = z.minCoeff(&minIdx);

    result.flatness = maxVal - minVal;
    result.minPoint = cloud->points[minIdx];
    result.maxPoint = cloud->points[maxIdx];
    result.pointCount = cloud->size();

    // RMSE
    float sumSq = 0.0f;
    float mean = z.mean();
    for (int i = 0; i < z.size(); ++i) {
        float d = z[i] - mean;
        sumSq += d * d;
    }
    result.rmse = std::sqrt(sumSq / z.size());

    result.valid = true;
    return result;
}

float flatnessRMSE(CP cloud)
{
    if (!cloud || cloud->empty()) return 0.0f;

    auto coeff = planeFitLSM(cloud);
    Eigen::Vector3f normal = Eigen::Vector3f(coeff[0], coeff[1], -1).normalized();
    Eigen::VectorXf z = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);

    float mean = z.mean();
    float sumSq = 0.0f;
    for (int i = 0; i < z.size(); ++i) {
        float d = z[i] - mean;
        sumSq += d * d;
    }
    return std::sqrt(sumSq / z.size());
}

float flatnessMinimumZone(CP cloud)
{
    if (!cloud || cloud->size() < 4) return 0.0f;

    // Build convex hull
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);
    CP hullCloud(new CloudT);
    hull.reconstruct(*hullCloud);

    if (hullCloud->size() < 3) return 0.0f;

    float minFlatness = std::numeric_limits<float>::max();
    int n = static_cast<int>(hullCloud->size());

    // Exhaustive search over all triplets
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            for (int k = j + 1; k < n; ++k) {
                // Compute plane normal from 3 points
                Eigen::Vector3f v1 = hullCloud->points[j].getVector3fMap() -
                                     hullCloud->points[i].getVector3fMap();
                Eigen::Vector3f v2 = hullCloud->points[k].getVector3fMap() -
                                     hullCloud->points[i].getVector3fMap();
                Eigen::Vector3f normal = v1.cross(v2);
                if (normal.norm() < 1e-10f) continue;
                normal.normalize();

                // Project hull points onto normal
                Eigen::VectorXf z = normal.transpose() * hullCloud->getMatrixXfMap(3, 4, 0);
                float flatness = z.maxCoeff() - z.minCoeff();

                if (flatness < minFlatness) {
                    minFlatness = flatness;
                }
            }
        }
    }

    return minFlatness;
}

// ---------------------------------------------------------------------------
// Plane extraction functions
// ---------------------------------------------------------------------------

CP extractLargestCluster(CP cloud, float tolerance,
                         int minClusterSize, int maxClusterSize)
{
    if (!cloud || cloud->empty()) return cloud;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance);
    if (minClusterSize > 0) ec.setMinClusterSize(minClusterSize);
    if (maxClusterSize > 0) ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    CP maxCluster(new CloudT);
    for (const auto& idx : clusterIndices) {
        CP cluster(new CloudT);
        for (const auto& i : idx.indices) {
            cluster->push_back(cloud->points[i]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        if (cluster->size() > maxCluster->size()) {
            maxCluster = cluster;
        }
    }
    return maxCluster;
}

CP extractPlane(CP cloud, const FlatnessConfig& config)
{
    if (!cloud || cloud->empty()) return cloud;

    // Z pass-through filter
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(config.zCenter - config.zTolerance,
                         config.zCenter + config.zTolerance);
    pass.filter(*cloud);

    // Largest cluster
    auto planeBig = extractLargestCluster(cloud, config.clusterTolerance);

    // Center
    PointT center;
    center.getVector3fMap() = planeBig->getMatrixXfMap(3, 4, 0).rowwise().mean();

    // Statistical distance filtering
    double totalDist = 0.0;
    for (const auto& p : planeBig->points) {
        double dx = p.x - center.x;
        double dy = p.y - center.y;
        double dz = p.z - center.z;
        totalDist += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    double meanDist = totalDist / planeBig->points.size();

    CP ret(new CloudT);
    for (const auto& p : planeBig->points) {
        double dx = p.x - center.x;
        double dy = p.y - center.y;
        double dz = p.z - center.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (std::abs(meanDist - dist) < config.boundaryTolerance) {
            ret->push_back(p);
        }
    }

    // KNN boundary removal
    float knnRadius = config.knnRadius * config.resolution;
    int minK = static_cast<int>(0.8f * config.knnRadius * config.knnRadius * 3.14f);
    ret = knnBoundaryRemove(ret, knnRadius, minK);

    return ret;
}

CP extractCircle(CP cloud, const FlatnessConfig& config)
{
    if (!cloud || cloud->empty()) return cloud;

    // Z pass-through filter
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(config.zCenter - config.zTolerance,
                         config.zCenter + config.zTolerance);
    pass.filter(*cloud);

    // Largest cluster
    auto planeBig = extractLargestCluster(cloud, config.clusterTolerance);

    // KNN boundary
    float knnRadius = config.knnRadius * config.resolution;
    int minK = static_cast<int>(0.5f * config.knnRadius * config.knnRadius * 3.14f);
    auto bound = knnBoundary(planeBig, knnRadius, minK);

    // RANSAC circle 3D fit
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(bound);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values.size() < 4) {
        return planeBig; // Fallback: return largest cluster
    }

    PointT circleCenter(coefficients->values[0],
                        coefficients->values[1],
                        coefficients->values[2]);
    double fittedRadius = coefficients->values[3];

    // Determine inner vs outer ring
    double sumDist = 0.0;
    for (const auto& p : cloud->points) {
        sumDist += pcl::euclideanDistance(p, circleCenter);
    }
    double meanDist = sumDist / cloud->points.size();

    CP ret(new CloudT);
    double refDistance;
    if (meanDist > fittedRadius) {
        refDistance = fittedRadius + config.circleRefOffset;
    } else {
        refDistance = fittedRadius - config.circleRefOffset;
    }

    for (const auto& p : planeBig->points) {
        double dx = p.x - circleCenter.x;
        double dy = p.y - circleCenter.y;
        double dz = p.z - circleCenter.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (std::abs(refDistance - dist) < config.circleTolerance) {
            ret->push_back(p);
        }
    }

    return ret;
}

} // namespace rxs
