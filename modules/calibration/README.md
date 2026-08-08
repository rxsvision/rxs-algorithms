# Calibration Module

## Status: SKELETON (source code lost)

Original calibration source code was lost. Only the compiled DLL (`docs/legacy_dll/calibration.dll`) and RD documentation survive.

## What exists

- **docs/** — Complete RD documentation (requirements, design, test specs, pseudocode, paper, SDK reference)
- **docs/legacy_dll/** — Compiled DLL and SDK doc (interface reference only)

## What needs to be done

1. Read `docs/RD-03002-001_算法源码.txt` and `docs/标定算法伪代码.pdf` for algorithm spec
2. Read `docs/RD-03002-003_算法函数使用说明(标定).docx` for SDK interface
3. Implement `calibrate()`, `computeError()`, `getTransform()` in `calibration.cpp`
4. Write `test_main.cpp` with test cases from `docs/RD-03001-005_算法功能测试表.xlsx`

## Algorithm overview (from RD docs)

- Two-cloud calibration: RANSAC coarse alignment + ICP fine registration
- Input: source + target point clouds
- Output: 4x4 rigid transformation matrix + RMS error
- See `docs/标定算法伪代码.pdf` for pseudocode

## License

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
