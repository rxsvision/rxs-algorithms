# rxs_coplanarity

点云共面性测量模块。

## 功能

- 基于 PCA 的基准平面法向量估计
- 评估平面沿法向量方向的高度偏差计算
- 支持多个基准平面和评估平面

## 算法

1. 从 ROI 区域提取并合并基准平面点云
2. PCA 拟合：最小特征值方向 = 基准平面法向量
3. 将评估平面点云投影到法向量方向
4. 偏差 = 评估平面平均高度 - 基准平面平均高度

## API

### `coplanarity()`
简单 API，返回各评估平面的高度偏差向量。

### `coplanarityEx()`
完整 API，返回 `CoplanarityResult`（偏差、法向量、基准高度）。

### ROIType

```cpp
rxs::ROIType roi{-102.4f, -98.3f, 48.29f, 51.2f, 100.0f, 200.0f};
// Layout: x_min, x_max, y_min, y_max, z_min, z_max
```

## 依赖

- PCL (common, io, filters)
- Eigen3

## 构建

```bash
mkdir build && cd build
cmake .. -DRXS_COPLANARITY_BUILD_TESTS=ON
cmake --build .
```

## 许可

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)