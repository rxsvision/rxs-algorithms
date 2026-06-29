/**
 * @file hole_depth.h
 * @brief Hole depth measurement algorithm — standalone SDK module
 *
 * Reconstructed from the original holeDeepth operator that was referenced
 * in protected_rxsToolKit via GetProcAddress("holeDeepth") but was never
 * exported from czxToolkit.dll.
 *
 * This implementation is based on the existing Hole class algorithms
 * (Hole.cpp/h/hpp in rxs-toolkit) which provide:
 *   - KNN-based boundary extraction
 *   - RANSAC ellipse fitting for hole center
 *   - Outlier removal for hole detection
 *
 * The module can be used:
 *   1. As a standalone library (compile with PCL only)
 *   2. Integrated into czxToolkit.dll (add to Source.def)
 *   3. Linked directly into consumer applications
 *
 * @author RXS Vision — Suzhou Ruixin Vision Technology Co., Ltd.
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

#ifdef _WIN32
    #ifdef HOLE_DEPTH_BUILD_DLL
        #define HOLE_DEPTH_API __declspec(dllexport)
    #else
        #define HOLE_DEPTH_API __declspec(dllimport)
    #endif
#else
    #define HOLE_DEPTH_API
#endif

// ---------------------------------------------------------------------------
// Type aliases (matching rxs-toolkit/types_.h)
// ---------------------------------------------------------------------------
typedef pcl::PointXYZ   PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr     CP;

// ---------------------------------------------------------------------------
// Result structures
// ---------------------------------------------------------------------------

/**
 * @brief Single hole depth measurement result
 */
struct HoleDepthResult {
    double centerX;       ///< Hole center X (mm)
    double centerY;       ///< Hole center Y (mm)
    double centerZ;       ///< Hole bottom Z (mm)
    double depth;         ///< Measured depth (mm), positive = hole goes down
    double radius;        ///< Estimated hole radius (mm)
    double surfaceZ;      ///< Surrounding surface Z (mm)
    bool   valid;         ///< Whether measurement succeeded
};

/**
 * @brief Configuration for hole depth measurement
 */
struct HoleDepthConfig {
    double searchRadius = 3.0;       ///< Radius for boundary search (mm)
    int    minNeighbors = 10;        ///< Min neighbors for outlier removal
    double voxelSize = 0.1;          ///< Voxel grid downsampling size (mm)
    double knnRadius = 1.0;          ///< KNN search radius for boundary extraction
    int    knnThreshold = 15;        ///< Max neighbors for boundary point classification
    double knnDiffThreshold = 0.05;  ///< Mean distance threshold for boundary detection
    double step = 1.0;               ///< Sampling step for depth profile (mm)
    double passFilterWidth = 6.0;    ///< X/Y pass-through filter width around hole center
    bool   enableVoxelGrid = true;   ///< Enable voxel grid downsampling
    bool   enableRANSAC = true;      ///< Enable RANSAC ellipse fitting for center

    HoleDepthConfig() = default;
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Measure hole depth from a point cloud
 *
 * This is the primary API function. It detects holes in the point cloud,
 * extracts boundaries, and measures the depth of each hole.
 *
 * @param cloud       Input point cloud (XYZ)
 * @param config      Configuration parameters
 * @return Vector of hole depth results (one per detected hole)
 */
HOLE_DEPTH_API std::vector<HoleDepthResult> measureHoleDepth(
    CP cloud,
    const HoleDepthConfig& config = HoleDepthConfig()
);

/**
 * @brief Measure hole depth with simple step parameter
 *
 * Compatibility wrapper matching the original holeDeepth signature pattern.
 * Uses default config with custom step.
 *
 * @param cloud  Input point cloud
 * @param step   Sampling step for depth profile (mm)
 * @return Vector of hole depth results
 */
HOLE_DEPTH_API std::vector<HoleDepthResult> measureHoleDepth(
    CP cloud,
    double step
);

/**
 * @brief Find hole centers in a point cloud
 *
 * Uses radius outlier removal to find sparse regions (holes) and
 * computes the centroid of each hole region.
 *
 * @param cloud        Input point cloud
 * @param searchRadius  Radius for outlier removal
 * @param minNeighbors  Min neighbors threshold
 * @return Vector of hole center points
 */
HOLE_DEPTH_API std::vector<PointT> findHoleCenters(
    CP cloud,
    double searchRadius = 3.0,
    int minNeighbors = 10
);

/**
 * @brief Extract hole boundary using KNN
 *
 * Classifies points as boundary points based on local neighborhood
 * density and mean displacement.
 *
 * @param cloud        Input point cloud
 * @param radius       KNN search radius
 * @param maxNeighbors  Max neighbors for boundary classification
 * @param diffThreshold Mean displacement threshold
 * @return Boundary point cloud
 */
HOLE_DEPTH_API CP extractHoleBoundary(
    CP cloud,
    double radius = 1.0,
    int maxNeighbors = 15,
    double diffThreshold = 0.05
);

/**
 * @brief Measure depth at a specific location
 *
 * Measures the Z-depth difference between the surrounding surface
 * and the lowest point within a region around the given center.
 *
 * @param cloud    Input point cloud
 * @param center   Hole center point
 * @param width    X/Y filter width around center
 * @param step     Sampling step for profile
 * @return Hole depth result
 */
HOLE_DEPTH_API HoleDepthResult measureDepthAtLocation(
    CP cloud,
    const PointT& center,
    double width = 6.0,
    double step = 1.0
);

/**
 * @brief Format hole depth results as a string table
 *
 * @param results  Hole depth results
 * @return Formatted string table
 */
HOLE_DEPTH_API std::string formatResults(
    const std::vector<HoleDepthResult>& results
);
