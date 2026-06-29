#include "flatness.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <cmath>

/**
 * @brief Test program for rxs::flatness module
 *
 * Usage:
 *   flatness_test [pcd_file] [config_file]
 *
 * Without arguments: generates a synthetic plane with known flatness
 * and verifies the measurement.
 */

using namespace rxs;

// Generate a synthetic plane with known flatness deviation
static CP generateTestPlane(float knownFlatness, int gridSize, float resolution)
{
    CP cloud(new CloudT);
    float halfSize = gridSize * resolution / 2.0f;

    for (int i = 0; i < gridSize; ++i) {
        for (int j = 0; j < gridSize; ++j) {
            PointT p;
            p.x = -halfSize + i * resolution;
            p.y = -halfSize + j * resolution;
            // Create a tilted plane with known peak-to-valley
            p.z = (p.x / halfSize) * knownFlatness * 0.5f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

int main(int argc, char** argv)
{
    if (argc >= 3) {
        // File mode: load PCD/PLY and measure flatness
        CP cloud(new CloudT);
        std::string path = argv[1];

        if (path.substr(path.find_last_of('.')) == ".pcd") {
            pcl::io::loadPCDFile(path, *cloud);
        } else {
            pcl::io::loadPLYFile(path, *cloud);
        }

        FlatnessConfig config = FlatnessConfig::fromProfile(argv[2]);
        std::cout << "Config: z=" << config.zCenter
                  << " resolution=" << config.resolution << std::endl;

        // Extract plane region
        auto plane = extractPlane(cloud, config);

        // Measure flatness
        auto result = flatnessEx(plane);
        if (result.valid) {
            std::cout << "Flatness (peak-to-valley): " << result.flatness << std::endl;
            std::cout << "RMSE:                      " << result.rmse << std::endl;
            std::cout << "Normal: [" << result.normal.transpose() << "]" << std::endl;
            std::cout << "Points: " << result.pointCount << std::endl;
            std::cout << "Min point: [" << result.minPoint.x << ", "
                      << result.minPoint.y << ", " << result.minPoint.z << "]" << std::endl;
            std::cout << "Max point: [" << result.maxPoint.x << ", "
                      << result.maxPoint.y << ", " << result.maxPoint.z << "]" << std::endl;
        } else {
            std::cout << "ERROR: " << result.error << std::endl;
        }

        // Optional: minimum zone flatness (slower but ISO-compliant)
        if (plane->size() < 5000) {
            float mz = flatnessMinimumZone(plane);
            std::cout << "Minimum zone flatness:     " << mz << std::endl;
        }

        return 0;
    }

    // Synthetic test mode
    std::cout << "=== Synthetic Plane Flatness Test ===" << std::endl;

    const float knownFlatness = 0.1f;  // 0.1 mm peak-to-valley
    const int gridSize = 50;
    const float resolution = 0.1f;

    CP cloud = generateTestPlane(knownFlatness, gridSize, resolution);
    std::cout << "Generated plane: " << cloud->size() << " points"
              << ", known flatness: " << knownFlatness << std::endl;

    // Test 1: Basic flatness (backward compatible API)
    PointT minP, maxP;
    float f1 = flatness(cloud, minP, maxP);
    std::cout << "\n[flatness]            P-V: " << f1
              << "  (expected: " << knownFlatness << ")"
              << "  error: " << std::abs(f1 - knownFlatness) / knownFlatness * 100 << "%"
              << std::endl;

    // Test 2: Extended flatness (with RMSE + normal)
    auto r2 = flatnessEx(cloud);
    if (r2.valid) {
        std::cout << "[flatnessEx]          P-V: " << r2.flatness
                  << "  RMSE: " << r2.rmse
                  << "  normal: [" << r2.normal.transpose() << "]"
                  << std::endl;
    }

    // Test 3: RMSE flatness
    float f3 = flatnessRMSE(cloud);
    std::cout << "[flatnessRMSE]        RMSE: " << f3 << std::endl;

    // Test 4: Minimum zone flatness
    float f4 = flatnessMinimumZone(cloud);
    std::cout << "[flatnessMinimumZone] MZ:  " << f4
              << "  (should be <= " << knownFlatness << ")"
              << std::endl;

    // Test 5: Validation
    auto r5 = flatnessEx(nullptr);
    std::cout << "\n[validation] Null cloud:  " << (r5.valid ? "FAIL" : "OK") << std::endl;

    CP empty(new CloudT);
    auto r6 = flatnessEx(empty);
    std::cout << "[validation] Empty cloud: " << (r6.valid ? "FAIL" : "OK") << std::endl;

    return 0;
}
