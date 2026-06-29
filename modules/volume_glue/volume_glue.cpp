#include "volume_glue.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <cmath>
#include <algorithm>

namespace rxs {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static float pointDistance(const PointT& p1, const PointT& p2)
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

CP getGlue(CP cloud, float planeZ, PointT gluePos,
           float clusterTolerance, int minClusterSize)
{
    if (!cloud || cloud->empty()) return cloud;

    // Pass-through filter: keep points above planeZ
    CP filtered(new CloudT);
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.z)) continue;
        if (p.z > planeZ) {
            filtered->push_back(p);
        }
    }
    if (filtered->empty()) return filtered;

    // Euclidean clustering
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(10000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(clusterIndices);

    // Find nearest cluster to gluePos
    float minDist = std::numeric_limits<float>::max();
    CP nearestCloud(new CloudT);

    for (const auto& idx : clusterIndices) {
        if (idx.indices.empty()) continue;

        // Use centroid of first few points as reference
        PointT refPoint = filtered->points[idx.indices[0]];
        float dist = pointDistance(refPoint, gluePos);

        if (dist < minDist) {
            minDist = dist;
            nearestCloud->clear();
            for (const auto& i : idx.indices) {
                nearestCloud->push_back(filtered->points[i]);
            }
        }
    }

    nearestCloud->width = nearestCloud->size();
    nearestCloud->height = 1;
    nearestCloud->is_dense = true;
    return nearestCloud;
}

double volumeGlue(CP glue, double dx, double dy, float planeZ)
{
    if (!glue || glue->empty()) return 0.0;

    double sum = 0.0;
    for (const auto& p : glue->points) {
        if (!std::isfinite(p.z)) continue;
        double h = p.z - planeZ;
        if (h > 0.0) {
            sum += dx * dy * h;
        }
    }
    return sum;
}

GlueResult measureGlue(CP cloud, double dx, double dy,
                       float planeZ, PointT gluePos)
{
    GlueResult result;
    result.valid = false;
    result.volume = 0.0;
    result.pointCount = 0;

    if (!cloud) {
        result.error = "Null cloud pointer";
        return result;
    }
    if (cloud->empty()) {
        result.error = "Empty cloud";
        return result;
    }
    if (dx <= 0 || dy <= 0) {
        result.error = "Invalid grid spacing";
        return result;
    }

    result.glue = getGlue(cloud, planeZ, gluePos);
    if (!result.glue || result.glue->empty()) {
        result.error = "No glue region found above plane Z=" + std::to_string(planeZ);
        return result;
    }

    result.volume = volumeGlue(result.glue, dx, dy, planeZ);
    result.pointCount = result.glue->size();
    result.valid = true;
    return result;
}

CP imgToCloud(const cv::Mat& img, float offX, float offY)
{
    CP ret(new CloudT);
    if (img.empty()) return ret;

    for (int r = 0; r < img.rows; ++r) {
        for (int c = 0; c < img.cols; ++c) {
            float z = img.at<float>(r, c);
            if (z == 0.0f || !std::isfinite(z)) continue;
            PointT p;
            p.x = c * offX;
            p.y = r * offY;
            p.z = z;
            ret->push_back(p);
        }
    }
    ret->width = ret->size();
    ret->height = 1;
    ret->is_dense = true;
    return ret;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr imgToCloudMasked(
    const cv::Mat& img, float offX, float offY,
    const cv::Mat& color, const cv::Mat& mask)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ret(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (img.empty()) return ret;

    for (int r = 0; r < img.rows; ++r) {
        for (int c = 0; c < img.cols; ++c) {
            if (!mask.empty() && mask.at<uchar>(r, c) == 255) continue;
            float z = img.at<float>(r, c);
            if (z == 0.0f || !std::isfinite(z)) continue;
            uchar gray = color.empty() ? 128 : color.at<uchar>(r, c);
            pcl::PointXYZRGB p(gray, gray, gray);
            p.x = c * offX;
            p.y = r * offY;
            p.z = z;
            ret->push_back(p);
        }
    }
    ret->width = ret->size();
    ret->height = 1;
    ret->is_dense = true;
    return ret;
}

cv::Mat getGlueMask(const cv::Mat& grayImg)
{
    if (grayImg.empty()) return cv::Mat();

    cv::Mat result = grayImg.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(result, result, kernel);
    cv::erode(result, result, kernel);

    cv::Mat mask;
    cv::threshold(result, mask, 20, 255, cv::THRESH_BINARY_INV);
    return mask;
}

cv::Mat tiffToDepth(const cv::Mat& img, float zRatio)
{
    cv::Mat result;
    img.convertTo(result, CV_32FC1);
    result = result * zRatio;
    return result;
}

} // namespace rxs
