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

---

## 算法管线状态 (Algorithm Pipeline Status)

理想胶量体积计算管线 vs 当前实现状态：

| 步骤 | 理想管线 | 当前实现 | 状态 |
|------|---------|---------|------|
| 1. PCA 主方向对齐 | 对点云做 PCA，将胶路主方向旋转到 XY 平面 | **开发过但未启用**。原始代码 `源.cpp` 第 325-327 行写了 PCA 代码但全部注释掉了 | ⚠️ 已注释 |
| 2. ROI 选择 | 框选或数值设定胶路区域 | **部分实现**。通过 `gluePos` 参数 + Z pass-through 粗略定位，无 2D ROI 框选 | ⚠️ 部分 |
| 3. 参考平面拟合 | SVD/LSM 拟合基底平面，确定 planeZ | **未实现**。使用外部传入的固定 `planeZ` 值，不做拟合 | ❌ 未开发 |
| 4. Z 通带滤波 | 保留 planeZ 上方的点 (胶路) | **已实现**。`getGlue()` 中 pass-through filter | ✅ 已实现 |
| 5. 欧几里得聚类 | 分离胶路连通域 | **已实现**。`getGlue()` 中聚类取最大簇 | ✅ 已实现 |
| 6. 3σ 统计离群去除 | 拟合后残差剔除 | **未实现** | ❌ 未开发 |
| 7. Z 方向高度积分 | V = Σ(dx × dy × (z_i - planeZ)) | **已实现**。`volumeGlue()` | ✅ 已实现 |
| 8. 深度图转点云 | TIFF → 深度值 → XYZ 点云 | **已实现**。`tiffToDepth()` + `imgToCloud()` | ✅ 已实现 |

### 已注释的 PCA 代码 (原始 `源.cpp` 第 325-327 行)

```cpp
//pcl::PCA<pcl::PointXYZ> pca;                    // ← 被注释
//pca.setInputCloud(cloud);                       // ← 被注释
//Eigen::Matrix3f eigenvectors = pca.getEigenVectors();  // ← 被注释
```

原开发者**打算做 PCA 对齐但未完成**。增强版中已清理这些死代码。

### 与 flatness 模块的对比

| 对比项 | Volume_glue | Flatness |
|--------|------------|---------|
| 参考平面 | 固定 planeZ (外部设定) | SVD 拟合 (数据驱动) |
| 法向量 | 不计算 | 从平面系数提取 |
| 滤波步骤 | 2 步 (Z + 聚类) | 4 步 (Z + 聚类 + 距离 + KNN) |
| 测量 | Z 方向高度积分 | 法向量方向偏差 |
| PCA | 写了但注释掉了 | 完全未使用 |

### 二次开发建议

- **如基底平面已知且固定**：当前固定 `planeZ` 方式可直接使用
- **如需自动拟合基底平面**：参考 flatness 模块的 `fitPlaneSVD()`，在 `getGlue()` 之前添加平面拟合步骤
- **如需 PCA 对齐**：取消注释原始 PCA 代码并补全旋转逻辑，或参考 `surface` 模块的 `constructRotationFromZ()`
- **如需 3σ 滤波**：在聚类之后、积分之前添加残差统计剔除步骤
