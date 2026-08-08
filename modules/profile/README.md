# Profile Module

## Status: SKELETON (source code lost)

Original profile source code was lost. Only the compiled DLL (`docs/legacy_dll/computeProfileUsingPSO.dll`) and RD documentation survive.

## What exists

- **docs/** — Complete RD documentation (requirements, design, test specs, SDK reference)
- **docs/legacy_dll/** — Compiled DLL (computeProfileUsingPSO.dll) and SDK doc
- **docs/profile_pso.7z** — Legacy archive (DLL + docs only, no source)

## What needs to be done

1. Read `docs/RD-03002-001_算法源码.txt` for algorithm overview
2. Read `docs/最小区域法轮廓度检测算子.docx` for minimum zone method spec
3. Read `docs/RD-03002-003_算法函数使用说明.docx` for SDK interface
4. Implement `computeProfile()`, `compareWithCAD()`, `getDeviation()` in `profile.cpp`
5. Write `test_main.cpp` with test cases from `docs/RD-03001-004_算法功能测试表.xlsx`

## Algorithm overview (from RD docs)

- Minimum zone method for profile evaluation
- PSO (Particle Swarm Optimization) variant available via legacy DLL
- Input: measured point cloud + nominal CAD reference
- Output: profile deviation (peak-to-valley), RMS, per-point distribution
- See `docs/最小区域法轮廓度检测算子.docx` for detailed algorithm

## License

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
