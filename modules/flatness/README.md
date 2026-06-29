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

---

## 算法管线状态 (Algorithm Pipeline Status)

理想平面度计算管线 vs 当前实现状态：

| 步骤 | 理想管线 | 当前实现 | 状态 |
|------|---------|---------|------|
| 1a. Z 通带滤波 | 按 Z 值范围裁剪，粗略去除高度离群 | **已实现**。`zCenter ± zTolerance` pass-through filter | ✅ 已实现 |
| 1b. 欧几里得聚类 | 取最大连通域，去除离散噪点块 | **已实现**。`extractLargestCluster()` | ✅ 已实现 |
| 1c. 距中心距离过滤 | 计算各点到点云中心距离，保留 `|meanDist - dist| < tol` 的点 | **已实现**。几何边界修剪 | ✅ 已实现 |
| 1d. KNN 边界移除 | KdTree 检查邻域密度，移除稀疏边缘点 | **已实现**。`removeBoundaryPoints()` | ✅ 已实现 |
| 1e. 3σ 统计离群去除 | LSM 拟合后计算残差，剔除 `|r_i - mean| > 3σ` 的点 | **未实现**。当前无残差统计滤波 | ❌ 未开发 |
| 2. 平面拟合 + 法向量 | SVD 最小二乘: `z = ax + by + d`, normal = (a, b, -1) | **已实现**。`fitPlaneSVD()` | ✅ 已实现 |
| 3a. 峰谷平面度 | 法向量方向 max - min 偏差 | **已实现**。`flatness()` / `flatnessEx()` | ✅ 已实现 |
| 3b. RMSE 平面度 | 法向量方向均方根偏差 | **已实现**。`flatnessRMSE()` | ✅ 已实现 |
| 3c. 最小区域法 | 凸包穷举搜索，两平行平面最小间距 | **已实现**。`flatnessMinimumZone()` (ISO 1101) | ✅ 已实现 |
| 4. 拟合后再滤波 | 拟合 → 残差剔除 → 重新拟合 → 取最大距离 | **未实现**。无迭代 refine 选项 | ❌ 未开发 |

### 已有但未调用的工具

| 工具 | 位置 | 说明 |
|------|------|------|
| `pcl::PCA` | PCL 内置 | `surface`、`comparison`、`minimum_zone` 模块使用了 PCA 对齐，但 flatness 未使用。当前平面拟合用 SVD 而非 PCA |
| `constructRotationFromZ()` | `czxTool.h` (rxs-toolkit) | 可将平面法向量旋转到 Z 轴，但 flatness 直接在原坐标系做 SVD 拟合 |

### 与 volume_glue 的对比

| 对比项 | Flatness | Volume_glue |
|--------|---------|------------|
| 参考平面 | SVD 拟合平面 (数据驱动) | 固定 planeZ 值 (外部设定) |
| 法向量 | 从平面系数提取 (a, b, -1) | 不计算 |
| 滤波方法 | 4 步几何滤波 (Z/聚类/距离/KNN) | 2 步 (Z pass-through + 聚类) |
| 测量方式 | 法向量方向偏差 | Z 方向高度积分 |
| 3σ 滤波 | ❌ 未实现 | ❌ 未实现 |

### 二次开发建议

- **当前管线对几何离群点较鲁棒**（4 步滤波），但对统计离群点无防护
- **如需 3σ 滤波**：在 `fitPlaneSVD()` 之后添加残差计算 + 3σ 剔除 + 重新拟合的迭代步骤
- **如需 PCA 对齐**：在 `extractPlane()` 之前添加 `pcl::PCA` 主方向旋转，参考 `surface` 模块
