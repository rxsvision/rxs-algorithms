#include "volume.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>

/**
 * @brief Test program for rxs::volume module
 *
 * Generates a hemisphere point cloud and verifies volume calculation
 * against the analytical formula: V_hemisphere = (2/3) * pi * r^3
 */

using namespace rxs;

int main()
{
    // Generate hemisphere point cloud
    const double radius = 1.0;
    const double xyInterval = 0.008;
    CP cloud(new CloudT);

    for (double x = -radius; x <= radius; x += xyInterval) {
        for (double y = -radius; y <= radius; y += xyInterval) {
            double z_sq = radius * radius - x * x - y * y;
            if (z_sq >= 0) {
                PointT p;
                p.x = static_cast<float>(x);
                p.y = static_cast<float>(y);
                p.z = static_cast<float>(std::sqrt(z_sq));
                cloud->push_back(p);
            }
        }
    }

    std::cout << "Point count: " << cloud->size() << std::endl;

    // Analytical hemisphere volume: V = (2/3) * pi * r^3
    double analytical = (2.0 / 3.0) * M_PI * radius * radius * radius;
    std::cout << "Analytical hemisphere volume: " << analytical << std::endl;

    // Test 1: Original API (backward compatible)
    double v1 = volume(cloud, xyInterval);
    std::cout << "[volume]            Riemann sum: " << v1
              << "  (error: " << std::abs(v1 - analytical) / analytical * 100 << "%)" << std::endl;

    // Test 2: Rectilinear grid with separate dx/dy
    auto r2 = volumeRect(cloud, xyInterval, xyInterval);
    if (r2.valid) {
        std::cout << "[volumeRect]        Riemann sum: " << r2.volume
                  << "  (error: " << std::abs(r2.volume - analytical) / analytical * 100 << "%)" << std::endl;
    }

    // Test 3: Volume above plane (glue style)
    auto r3 = volumeAbovePlane(cloud, xyInterval, xyInterval, 0.0);
    if (r3.valid) {
        std::cout << "[volumeAbovePlane]  Above Z=0:  " << r3.volume
                  << "  (error: " << std::abs(r3.volume - analytical) / analytical * 100 << "%)" << std::endl;
    }

    // Test 4: Convex hull volume
    auto r4 = volumeConvexHull(cloud);
    if (r4.valid) {
        std::cout << "[volumeConvexHull]  Hull volume: " << r4.volume
                  << "  (error: " << std::abs(r4.volume - analytical) / analytical * 100 << "%)" << std::endl;
    } else {
        std::cout << "[volumeConvexHull]  FAILED: " << r4.error << std::endl;
    }

    // Test 5: Null/empty validation
    auto r5 = volumeRect(nullptr, 0.01, 0.01);
    std::cout << "[validation]        Null cloud:  " << (r5.valid ? "PASS(should fail)" : "OK") << std::endl;

    CP empty(new CloudT);
    auto r6 = volumeRect(empty, 0.01, 0.01);
    std::cout << "[validation]        Empty cloud: " << (r6.valid ? "PASS(should fail)" : "OK") << std::endl;

    auto r7 = volumeRect(cloud, -1.0, 0.01);
    std::cout << "[validation]        Bad spacing: " << (r7.valid ? "PASS(should fail)" : "OK") << std::endl;

    return 0;
}
