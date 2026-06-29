/**
 * @file test_main.cpp
 * @brief Test program for hole_depth module
 *
 * Usage:
 *   hole_depth_test <input.pcd> [step]
 *
 * Loads a PCD file, runs hole depth measurement, prints results.
 */

#include "hole_depth.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> [step]" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    double step = 1.0;
    if (argc >= 3) {
        step = std::stod(argv[2]);
    }

    // Load point cloud
    CP cloud(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(inputFile, *cloud) < 0) {
        std::cerr << "Error: Failed to load " << inputFile << std::endl;
        return 1;
    }
    std::cout << "Loaded " << cloud->size() << " points from " << inputFile << std::endl;

    // Measure hole depth
    std::cout << "Running hole depth measurement (step=" << step << ")..." << std::endl;
    auto results = measureHoleDepth(cloud, step);

    // Print results
    std::cout << std::endl << formatResults(results) << std::endl;

    return 0;
}
