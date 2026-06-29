# Flatness Module

> Point cloud flatness measurement — LSM, RMSE, and Minimum Zone methods

## API

### Core Measurement

| Function | Method | Standard | Complexity |
|----------|--------|----------|------------|
| `flatness(cloud, minP, maxP)` | LSM + Peak-to-Valley | Basic | O(n) |
| `flatnessEx(cloud)` | LSM + P-V + RMSE + Normal | Extended | O(n) |
| `flatnessRMSE(cloud)` | LSM + RMSE | Statistical | O(n) |
| `flatnessMinimumZone(cloud)` | Convex Hull + Exhaustive | ISO 1101 | O(h^3) |

### Plane Extraction

| Function | Description |
|----------|-------------|
| `extractPlane(cloud, config)` | Z-filter + clustering + statistical |
| `extractCircle(cloud, config)` | Z-filter + clustering + RANSAC circle |
| `extractLargestCluster(cloud, tol)` | Euclidean clustering |

## Usage

```cpp
#include "flatness.h"
using namespace rxs;

// Basic flatness (backward compatible)
PointT minP, maxP;
float f = flatness(cloud, minP, maxP);

// Extended result
auto result = flatnessEx(cloud);
if (result.valid) {
    std::cout << "Flatness: " << result.flatness << std::endl;
    std::cout << "RMSE: " << result.rmse << std::endl;
    std::cout << "Normal: " << result.normal.transpose() << std::endl;
}

// ISO 1101 minimum zone (for small clouds)
float mz = flatnessMinimumZone(cloud);

// Extract plane from noisy cloud
FlatnessConfig config;
config.zCenter = 5.0f;
config.resolution = 0.1f;
auto plane = extractPlane(cloud, config);
```

## Configuration

```cpp
struct FlatnessConfig {
    float zCenter = 0.0f;           // Z filter center
    float zTolerance = 2.0f;        // Z filter half-range
    float clusterTolerance = 0.2f;  // Clustering distance
    int knnRadius = 5;              // KNN search radius multiplier
    float resolution = 0.1f;        // Point cloud resolution
    float boundaryTolerance = 0.5f; // Boundary extraction tolerance
    float circleRefOffset = 0.4f;   // Circle reference offset
    float circleTolerance = 0.2f;   // Circle fitting tolerance
};
```

Load from config file:
```cpp
auto config = FlatnessConfig::fromProfile("flatness_conf.czx");
```

Config file format:
```
z=5.0
resolution=0.1
```

## Build

```bash
mkdir build && cd build
cmake .. -DRXS_FLATNESS_BUILD_TESTS=ON
cmake --build .
```

## Changes from Original

| Issue | Original | Enhanced |
|-------|----------|----------|
| Global variables | `conf`, `file` at file scope | `FlatnessConfig` parameter |
| `void main()` | Non-standard | `int main()` |
| Magic numbers | Hardcoded `0.2`, `0.4`, `5`, etc. | Named config fields |
| Missing dependency | `../czxDependence/czxTool.h` | Self-contained, PCL only |
| nanoflann | External dependency | PCL KdTreeFLANN (built-in) |
| No CMake | .sln/.vcxproj only | CMakeLists.txt |
| No validation | Crashes on null/empty | Returns error result |
| No RMSE | Peak-to-valley only | P-V + RMSE + Normal |
| No MZ method | LSM only | LSM + Minimum Zone (ISO 1101) |

## Dependencies

- PCL (common, io, filters, segmentation, surface, kdtree, search)
- Eigen3 (via PCL)
