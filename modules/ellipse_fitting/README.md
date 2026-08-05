# rxs_ellipse_fitting

椭圆拟合模块 — Taubin AMS + 两步LM非线性优化

## 算法

1. RANSAC平面分割 → 提取非平面点 → voxel降采样
2. 投影到2D平面 → 圆RANSAC分割多个椭圆
3. Taubin AMS代数拟合 → LM_algebraic(代数残差) → LM_sampson(Sampson距离)
4. 近圆约束(a/b>0.95时theta=0)

### 算法来源

从 `D:\01_Environments_AImodels\benchmark\main.cpp` (benchmark v6) 提取，保持算法逻辑完全一致。

### 关键改进（相对自研CzxRansac）

- Taubin AMS替代Fitzgibbon广义特征值：更稳健，不需要广义特征值分解
- 两步LM精修：代数残差(解析Jacobian)快速收敛 + Sampson距离(数值Jacobian)几何精修
- 圆RANSAC初值优先：分割阶段已找到准确圆心和半径，直接作为LM初值
- 近圆约束：a/b>0.95时theta无意义，强制为0

## 性能(benchmark v6)

| 指标 | 值 |
|---|---|
| 精度 E1 | 0.0217mm |
| 精度 E2 | 0.0284mm |
| 工业级标准 | <0.1mm |
| 耗时 | 118ms (611万点) |
| 鲁棒性 | 20次随机种子100%成功 |

## 接口

### 高层接口

```cpp
#include "ellipse_fitting.h"

rxs::EllipseParams params;
rxs::EllipseResult result = rxs::fitEllipsesOnPlane(cloud, params);
// result.centers — 3D椭圆中心
// result.ellipses — 2D椭圆参数
// result.plane_normal, result.plane_centroid — 拟合平面
```

### 底层接口

```cpp
// 仅2D拟合（已知2D点集）
rxs::Ellipse2D e = rxs::fitEllipse2D(pts2d, true, true, 0.95f);

// 仅Taubin代数拟合
Eigen::Matrix<float, 6, 1> coeffs;
bool ok = rxs::fitEllipseTaubin(pts2d, coeffs);

// 仅LM精修
rxs::refineEllipseLM_algebraic(pts2d, center, a, b, theta);
rxs::refineEllipseLM_sampson(pts2d, center, a, b, theta);
```

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| height_threshold | 0.06 | 非平面点高度阈值 |
| voxel_leaf | 0.02 | 体素降采样叶大小 |
| circle_r_min | 1.5 | 圆RANSAC最小半径 |
| circle_r_max | 2.5 | 圆RANSAC最大半径 |
| circle_dist_threshold | 0.02 | 圆RANSAC距离阈值 |
| circle_max_iter | 5000 | 圆RANSAC最大迭代 |
| enable_lm_algebraic | true | 启用代数残差LM |
| enable_lm_sampson | true | 启用Sampson距离LM |
| near_circle_ratio | 0.95 | 近圆约束阈值 |
| seed | 42 | RANSAC随机种子(0=随机) |

## 依赖

- PCL >= 1.14 (common, io, filters, sample_consensus, segmentation)
- Eigen3 >= 3.4

## 构建

```bash
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake \
    -DRXS_BUILD_ELLIPSE_FITTING=ON -DRXS_ELLIPSE_FITTING_BUILD_TESTS=ON
cmake --build build --config Release --target ellipse_fitting_test
```

## 历史

替换自研CzxRansac（拟合失败+设计缺陷），基于benchmark v6验证。
