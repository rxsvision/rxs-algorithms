#include "coplanarity.h"
#include <pcl/io/pcd_io.h>
#include <iostream>

int main()
{
    rxs::CP cloud(new rxs::CloudT);
    if (pcl::io::loadPCDFile<rxs::PointT>("1.pcd", *cloud) < 0) {
        std::cerr << "Failed to load 1.pcd" << std::endl;
        return 1;
    }

    std::vector<rxs::ROIType> ref_planes = {
        {-102.4f, -98.3f, 48.29f, 51.2f, 100.0f, 200.0f},
        {-102.4f, -98.3f, 56.4f,  60.0f, 100.0f, 200.0f}
    };

    std::vector<rxs::ROIType> estimate_planes = {
        {-102.4f, -98.3f, 68.1f, 70.8f, 100.0f, 200.0f},
        {-102.4f, -98.3f, 56.4f, 60.0f, 100.0f, 200.0f}
    };

    auto result = rxs::coplanarityEx(cloud, ref_planes, estimate_planes);
    if (!result.valid) {
        std::cerr << "Error: " << result.error << std::endl;
        return 1;
    }

    std::cout << "Reference normal: ("
              << result.normal.x() << ", "
              << result.normal.y() << ", "
              << result.normal.z() << ")" << std::endl;
    std::cout << "Reference Z: " << result.refZ << std::endl;
    std::cout << "Deviations:" << std::endl;
    for (size_t i = 0; i < result.deviations.size(); ++i) {
        std::cout << "  Plane " << i << ": " << result.deviations[i] << std::endl;
    }

    return 0;
}