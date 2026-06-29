# Volume Module

> Point cloud volume calculation — 4 computation strategies

## API

| Function | Description | Use Case |
|----------|-------------|----------|
| `volume(cloud, xyInterval)` | Riemann sum, uniform grid | Original API, backward compatible |
| `volumeRect(cloud, dx, dy)` | Riemann sum, separate dx/dy | Non-square grid spacing |
| `volumeAbovePlane(cloud, dx, dy, planeZ)` | Volume above reference Z | Glue height, material volume |
| `volumeConvexHull(cloud)` | Convex hull tetrahedron | Irregular boundaries, unknown spacing |
| `volumeConvexHullAbovePlane(cloud, planeZ)` | Convex hull above Z | Irregular + reference plane |

## Usage

```cpp
#include "volume.h"
using namespace rxs;

// Original API (backward compatible)
double v = volume(cloud, 0.008);

// Enhanced: separate spacing + validation
auto result = volumeRect(cloud, 0.004, 0.006);
if (result.valid) {
    std::cout << "Volume: " << result.volume << std::endl;
}

// Glue volume above reference plane
auto glue = volumeAbovePlane(cloud, 0.004, 0.004, 2.5);

// Irregular boundary
auto hull = volumeConvexHull(cloud);
```

## Build

```bash
mkdir build && cd build
cmake .. -DRXS_VOLUME_BUILD_TESTS=ON
cmake --build .
```

## Returns

All enhanced APIs return `VolumeResult`:
```cpp
struct VolumeResult {
    double volume;       // Computed volume
    size_t pointCount;   // Points used
    bool valid;          // Success flag
    std::string error;   // Error message if !valid
};
```

## Validation

- Null pointer check
- Empty cloud check
- NaN/Inf point filtering
- Grid spacing positivity check
- Minimum point count (convex hull: 4+)
