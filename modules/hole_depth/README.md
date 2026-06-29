# Hole Depth Module

> 孔深测量算法 — 独立 SDK 模块

## 背景

`holeDeepth` 最初作为 `czxToolkit.dll` 的导出函数被消费者代码引用，但 DLL 从未实现/导出此函数（`GetProcAddress` 返回 NULL）。消费者侧 `protected_rxsToolKit.cpp` 中有完整的调用包装器，包括声明、注册、switch case 和实现，但所有调用都会失败。

Phase 1 修复时移除了消费者侧的全部 24 处引用（3 个版本 × 2 目录 × 4 处引用）。本模块将 `holeDeepth` 重建为**独立算法模块**，不依赖 DLL 基础设施。

## 算法来源

基于 `rxs-toolkit` 中已有的孔检测算法：

| 源文件 | 算法 | 本模块对应 |
|--------|------|-----------|
| `Hole.hpp` findHole() | 半径离群点去除检测孔区域 | `removeDensePoints()` |
| `Hole.hpp` removeRectangle() | 移除矩形边框伪影 | `removeRectangle()` |
| `Hole.cpp` KNNBoundExtract() | KNN 边界提取 | `knnBoundaryExtract()` |
| `Hole.hpp` centerFit() | RANSAC 椭圆拟合中心 | `computeCentroid()` |
| `Hole.hpp` Hull() | 凸包分层 | `clusterHoles()` |

## 算法流程

```
输入点云
    │
    ▼
[1] 体素下采样 (可选)
    │
    ▼
[2] 移除矩形边框伪影
    │
    ▼
[3] 半径离群点去除 → 稀疏点 (孔区域)
    │
    ▼
[4] 欧氏聚类 → 分离独立孔
    │
    ▼
[5] 对每个孔:
    ├── 计算孔中心 (质心)
    ├── PassThrough 过滤提取局部区域
    ├── KNN 边界提取
    ├── 表面 Z = 边界点 Z 中位数
    ├── 底部 Z = 孔内点 Z 10% 分位数
    ├── 深度 = 表面Z - 底部Z
    └── 半径 = 边界点到中心平均距离
    │
    ▼
输出: vector<HoleDepthResult>
```

## 使用方式

### 方式 1: 独立编译 (推荐)

```bash
mkdir build && cd build
cmake .. -DPCL_DIR=/path/to/pcl
cmake --build .
```

```cpp
#include "hole_depth.h"

CP cloud(new CloudT);
// ... load point cloud ...

auto results = measureHoleDepth(cloud, 1.0);  // step = 1.0mm
std::cout << formatResults(results);
```

### 方式 2: 集成到 czxToolkit.dll

1. 将 `hole_depth.h` 和 `hole_depth.cpp` 复制到 rxs-toolkit 源码目录
2. 将 `dll_holeDeepth.cpp` 添加到 DLL 项目
3. 在 `Source.def` 的 EXPORTS 中添加 `holeDeepth`
4. 重新编译 `czxToolkit.dll`

此后消费者侧的 `GetProcAddress("holeDeepth")` 将正常返回函数指针。

### 方式 3: 直接链接 (header-only 风格)

将 `hole_depth.h` 和 `hole_depth.cpp` 直接添加到消费者项目中编译，无需 DLL。

## API

### 主函数

```cpp
// 完整配置版本
std::vector<HoleDepthResult> measureHoleDepth(
    CP cloud,
    const HoleDepthConfig& config
);

// 简化版本 (仅指定 step)
std::vector<HoleDepthResult> measureHoleDepth(
    CP cloud,
    double step
);
```

### 辅助函数

```cpp
// 查找孔中心
std::vector<PointT> findHoleCenters(CP cloud, double searchRadius, int minNeighbors);

// 提取孔边界
CP extractHoleBoundary(CP cloud, double radius, int maxNeighbors, double diffThreshold);

// 测量指定位置的深度
HoleDepthResult measureDepthAtLocation(CP cloud, const PointT& center, double width, double step);

// 格式化结果
std::string formatResults(const std::vector<HoleDepthResult>& results);
```

### 结果结构

```cpp
struct HoleDepthResult {
    double centerX;    // 孔中心 X (mm)
    double centerY;    // 孔中心 Y (mm)
    double centerZ;    // 孔底部 Z (mm)
    double depth;      // 深度 (mm), 正值 = 向下
    double radius;     // 孔半径 (mm)
    double surfaceZ;   // 周围表面 Z (mm)
    bool   valid;      // 测量是否成功
};
```

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| searchRadius | 3.0 mm | 离群点去除搜索半径 |
| minNeighbors | 10 | 离群点去除最小邻居数 |
| voxelSize | 0.1 mm | 体素下采样大小 |
| knnRadius | 1.0 mm | KNN 搜索半径 |
| knnThreshold | 15 | KNN 边界分类最大邻居数 |
| knnDiffThreshold | 0.05 | KNN 均值位移阈值 |
| step | 1.0 mm | 深度剖面采样步长 |
| passFilterWidth | 6.0 mm | 孔中心周围过滤宽度 |
| enableVoxelGrid | true | 启用体素下采样 |
| enableRANSAC | true | 启用 RANSAC 椭圆拟合 |

## 依赖

- PCL >= 1.12 (common, io, filters, kdtree, segmentation)
- C++17
- Eigen3

## 许可证

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
