# RXS Algorithms

> 算法 Monorepo — 9 个算子模块

## 模块清单

| # | 模块 | 目录来源 | 功能 |
|---|------|---------|------|
| 1 | Regis | D:\2_4\Regis_v2 | 点云配准 (ICP + 独有 gatherCenter) |
| 2 | surface | D:\2_4\surface | NURBS 曲面拟合 |
| 3 | MinimumZone | D:\2_4\MinimumZone | 最小区域法平面度 |
| 4 | curveLength | D:\2_4\curveLength | 曲线长度计算 |
| 5 | comparison | D:\2_4\comparison | 3D 比对 |
| 6 | calibration | D:\2_4\3dmeasure_czx | 标定算法 |
| 7 | cylinder | D:\2_4\volume | 圆柱体计算 |
| 8 | Flatness | D:\2_4\Flatness | 平面度 (独立实现) |
| 9 | volume | D:\2_4\volumeGlue | 体积/胶量计算 |

## 算子查漏补缺状态

Phase 0 已完成自研算子 vs PCL/Open3D 对比分析：
- 11 个独有算法 (PCL/Open3D 无对应)
- 2 个薄包装 (computeConvexHull=直接调PCL, registration=独有+SVD调PCL)
- 2 个建议完善 (volume→凸包积分, Flatness→RMSE) — **Phase 2 已完成**
- 1 个幽灵引用 (holeDeepth, 已于 2026-06-29 从所有消费者代码中清除，并重建为独立模块)

## Phase 2 算子增强 (2026-06-29)

### volume 模块
- 新增 `volume.h` / `volume.cpp` — namespace rxs 现代化 API
- 4 种计算策略: `volume` / `volumeRect` / `volumeAbovePlane` / `volumeConvexHull`
- `VolumeResult` 结构体: 含 volume + pointCount + valid + error
- 输入验证: 空指针/空点云/NaN/Inf/网格间距检查
- CMakeLists.txt (PCL only, 可选测试)
- test_main.cpp: 半球体积验证 (解析解 vs 数值解)

### flatness 模块
- 新增 `flatness.h` / `flatness.cpp` — namespace rxs 现代化 API
- 3 种平面度方法: `flatness` (LSM P-V) / `flatnessRMSE` (LSM RMSE) / `flatnessMinimumZone` (ISO 1101)
- `FlatnessResult` 结构体: 含 flatness + rmse + normal + minPoint + maxPoint
- `FlatnessConfig` 结构体: 替代全局变量, 支持配置文件加载
- 平面提取: `extractPlane` / `extractCircle` / `extractLargestCluster`
- 消除外部依赖: nanoflann → PCL KdTreeFLANN, czxDependence → 自包含
- CMakeLists.txt (PCL only, 可选测试)
- test_main.cpp: 合成平面验证 + 文件模式

### volume_glue 模块
- 新增 `volume_glue.h` / `volume_glue.cpp` — namespace rxs 现代化 API
- `GlueResult` 结构体: 含 glue cloud + volume + pointCount
- `measureGlue` 全流程: 提取 + 体积计算一步到位
- 图像处理: `imgToCloud` / `imgToCloudMasked` / `getGlueMask` / `tiffToDepth`
- 消除全局变量 `conf`, 参数显式传递
- CMakeLists.txt (PCL + OpenCV)

## 许可证

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)

## 公司

苏州锐新视科技有限公司
