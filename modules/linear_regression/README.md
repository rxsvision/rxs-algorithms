# rxs_linear_regression

多元线性回归模块（最小二乘法 + LDLT 分解）。

## 功能

- 多元线性回归拟合：`w = (X^T X)^(-1) X^T y`
- LDLT 分解求解（数值稳定）
- 权重序列化/反序列化（逗号分隔字符串）
- CSV 数据读取工具

## API

### `LinearRegression::fit(X, y)`
训练模型，计算权重向量（含偏置项）。

### `LinearRegression::predict(X)`
使用训练好的模型进行预测。

### `LinearRegression::saveWeights() / loadWeights(str)`
权重序列化为逗号分隔字符串，便于持久化存储。

### `readCSV(filename, data, labels)`
从 CSV 文件读取数据（首行为表头跳过，末列为标签）。

## 依赖

- Eigen3（仅头文件，不需要 PCL）

## 构建

```bash
mkdir build && cd build
cmake .. -DRXS_LINEAR_REGRESSION_BUILD_TESTS=ON
cmake --build .
```

## 测试

```bash
./linear_regression_test
```

测试用例验证 `y = 2*x1 + 3*x2 + 1` 的回归拟合。

## 许可

BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)