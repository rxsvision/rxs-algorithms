# Volume Glue Module

> Glue volume measurement — extraction, clustering, and volume computation

## API

| Function | Description |
|----------|-------------|
| `getGlue(cloud, planeZ, gluePos)` | Extract glue region above plane |
| `volumeGlue(glue, dx, dy, planeZ)` | Compute glue volume |
| `measureGlue(cloud, dx, dy, planeZ, gluePos)` | Full pipeline (extract + measure) |
| `imgToCloud(img, offX, offY)` | Depth image to point cloud |
| `imgToCloudMasked(img, offX, offY, color, mask)` | Depth image to colored cloud with mask |
| `getGlueMask(grayImg)` | Generate glue mask from grayscale |
| `tiffToDepth(img, zRatio)` | Convert TIFF to depth values |

## Usage

```cpp
#include "volume_glue.h"
using namespace rxs;

// Full pipeline
float planeZ = 2.5f;
PointT gluePos(5, 3, 2);
auto result = measureGlue(cloud, 0.004, 0.004, planeZ, gluePos);
if (result.valid) {
    std::cout << "Glue volume: " << result.volume << " mm^3" << std::endl;
    std::cout << "Glue points: " << result.pointCount << std::endl;
}

// Step by step
auto glue = getGlue(cloud, planeZ, gluePos);
double vol = volumeGlue(glue, 0.004, 0.004, planeZ);

// From depth image
auto depth = tiffToDepth(rawImg, 0.0001f);
auto cloud = imgToCloud(depth, 0.004f, 0.004f);
```

## Changes from Original

| Issue | Original | Enhanced |
|-------|----------|----------|
| `void main()` | Non-standard return | Removed (library only) |
| Hardcoded params | `0.02`, `1000` in code | Configurable parameters |
| Global `conf` | File-scope global | Parameters passed explicitly |
| Commented code | ~50 lines of dead code | Cleaned |
| No validation | Crashes on null | Error checking + GlueResult |
| No CMake | .sln/.vcxproj | CMakeLists.txt |
| No header | All in single .cpp | Separate .h/.cpp |

## Dependencies

- PCL (common, io, segmentation, kdtree, search)
- OpenCV (core, imgproc)
- Eigen3 (via PCL)
