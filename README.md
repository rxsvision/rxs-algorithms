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
- 2 个建议完善 (volume→凸包积分, Flatness→RMSE)
- 1 个幽灵引用 (holeDeepth, 已于 2026-06-29 从所有消费者代码中清除)

## 许可证

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)

## 公司

苏州锐新视科技有限公司
