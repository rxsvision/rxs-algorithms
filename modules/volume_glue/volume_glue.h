#ifndef RXS_VOLUME_GLUE_H
#define RXS_VOLUME_GLUE_H

/**
 * @file volume_glue.h
 * @brief Glue volume measurement module
 *
 * Provides glue detection and volume measurement from point clouds:
 * - getGlue():       Extract glue region above a reference plane
 * - volumeGlue():    Compute glue volume
 * - imgToCloud():    Convert depth image to point cloud
 * - getGlueMask():   Generate glue mask from grayscale image
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include <string>

namespace rxs {

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CP;

/**
 * @brief Glue extraction result
 */
struct GlueResult {
    CP glue;            ///< Extracted glue point cloud
    double volume;      ///< Computed glue volume
    size_t pointCount;  ///< Points in glue region
    bool valid;         ///< Whether extraction succeeded
    std::string error;  ///< Error message if !valid
};

/**
 * @brief Extract glue region from point cloud
 *
 * Filters points above planeZ, performs Euclidean clustering,
 * and returns the cluster nearest to the reference position.
 *
 * @param cloud     Input point cloud
 * @param planeZ    Reference plane Z height (points above this are candidates)
 * @param gluePos   Reference position for nearest cluster selection
 * @param clusterTolerance  Clustering distance tolerance (default: 0.02)
 * @param minClusterSize    Minimum cluster size (default: 1000)
 * @return Extracted glue point cloud
 */
CP getGlue(CP cloud, float planeZ, PointT gluePos,
           float clusterTolerance = 0.02f,
           int minClusterSize = 1000);

/**
 * @brief Compute glue volume above a reference plane
 *
 * V = sum(dx * dy * max(0, z_i - planeZ)) for each point in glue cloud
 *
 * @param glue    Glue point cloud
 * @param dx      X-direction grid spacing
 * @param dy      Y-direction grid spacing
 * @param planeZ  Reference plane Z height
 * @return Volume value
 */
double volumeGlue(CP glue, double dx, double dy, float planeZ);

/**
 * @brief Full glue measurement pipeline
 *
 * Combines getGlue() + volumeGlue() in one call.
 *
 * @param cloud     Input point cloud
 * @param dx        X grid spacing
 * @param dy        Y grid spacing
 * @param planeZ    Reference plane Z height
 * @param gluePos   Reference position for cluster selection
 * @return GlueResult with extracted cloud and volume
 */
GlueResult measureGlue(CP cloud, double dx, double dy,
                       float planeZ, PointT gluePos);

/**
 * @brief Convert depth image to point cloud
 *
 * @param img    Depth image (CV_32FC1, z values in mm)
 * @param offX   X-direction pixel spacing (mm/pixel)
 * @param offY   Y-direction pixel spacing (mm/pixel)
 * @return Point cloud (zero-depth pixels skipped)
 */
CP imgToCloud(const cv::Mat& img, float offX, float offY);

/**
 * @brief Convert depth image to colored point cloud with mask
 *
 * @param img    Depth image (CV_32FC1)
 * @param offX   X-direction pixel spacing
 * @param offY   Y-direction pixel spacing
 * @param color  Grayscale color image
 * @param mask   Mask image (255 = skip pixel)
 * @return Colored point cloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr imgToCloudMasked(
    const cv::Mat& img, float offX, float offY,
    const cv::Mat& color, const cv::Mat& mask);

/**
 * @brief Generate glue mask from grayscale image
 *
 * Morphological close + threshold to identify glue regions.
 *
 * @param grayImg  Grayscale image
 * @return Binary mask (255 = glue region)
 */
cv::Mat getGlueMask(const cv::Mat& grayImg);

/**
 * @brief Convert SX TIFF depth image to depth values
 *
 * @param img     Input image (any integer type)
 * @param zRatio  Conversion ratio (z = pixel_value * zRatio)
 * @return Depth image (CV_32FC1)
 */
cv::Mat tiffToDepth(const cv::Mat& img, float zRatio);

} // namespace rxs

#endif // RXS_VOLUME_GLUE_H
