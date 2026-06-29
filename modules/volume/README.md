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

---

## 算法管线状态 (Algorithm Pipeline Status)

理想体积计算管线 vs 当前实现状态：

| 步骤 | 理想管线 | 当前实现 | 状态 |
|------|---------|---------|------|
| 1. PCA 主方向对齐 | 对点云做 PCA，将主方向旋转到 XY 平面，确保 Z 轴为高度方向 | **未实现**。代码假设输入点云已预对齐，Z 即为高度方向 | ❌ 未开发 |
| 2. 2D ROI 选择 | 通过框选或数值设定裁剪 XY 区域，限定积分范围 | **未实现**。假设输入点云已裁剪到目标区域 | ❌ 未开发 |
| 3. Z 方向高度积分 | Riemann sum: V = Σ(dx × dy × z_i) | **已实现**。`volume()` / `volumeRect()` | ✅ 已实现 |
| 3a. 参考平面扣除 | V = Σ(dx × dy × (z_i - planeZ)) | **已实现**。`volumeAbovePlane()` | ✅ 已实现 |
| 3b. 凸包四面体法 | ConvexHull → 四面体体积求和 | **已实现**。`volumeConvexHull()` | ✅ 已实现 |

### 已有但未调用的工具

| 工具 | 位置 | 说明 |
|------|------|------|
| `constructRotationFromZ()` | `czxTool.h` (rxs-toolkit) | 可将点云旋转到 Z 轴对齐，但 volume 模块从未调用。原开发者在 `surface`、`curve_length` 等模块中使用过 |
| `pcl::PCA` | PCL 内置 | 代码库中 `surface`、`comparison`、`minimum_zone` 模块使用了 PCA，但 volume 没有 |

### 二次开发建议

- **如输入点云已预对齐**（由 AOI Tester 应用层完成裁剪和旋转）：当前实现可直接使用
- **如需在模块内完成完整管线**：需自行添加 PCA 对齐和 ROI 裁剪步骤，参考 `surface` 模块中的 `constructRotationFromZ()` 用法
