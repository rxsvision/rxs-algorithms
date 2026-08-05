#ifndef RXS_LINEAR_REGRESSION_H
#define RXS_LINEAR_REGRESSION_H

/**
 * @file linear_regression.h
 * @brief Multivariate linear regression using least squares (Eigen LDLT)
 *
 * Solves w = (X^T X)^(-1) X^T y with bias term via LDLT decomposition.
 * Supports fit/predict, weight serialization (comma-separated string),
 * and CSV data loading.
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace rxs {

/**
 * @brief Multivariate linear regression via least squares
 *
 * Model: y = w0 + w1*x1 + w2*x2 + ... + wn*xn
 * Bias (w0) is added internally as a constant column of 1s.
 */
class LinearRegression {
public:
    LinearRegression() = default;

    /** @brief Fit model: compute weights via normal equation + LDLT */
    void fit(const std::vector<std::vector<float>>& X, const std::vector<float>& y);

    /** @brief Predict outputs for given inputs */
    std::vector<float> predict(const std::vector<std::vector<float>>& X);

    /** @brief Get current weights (including bias as first element) */
    std::vector<float> getWeights() const;

    /** @brief Serialize weights to comma-separated string */
    std::string saveWeights();

    /** @brief Deserialize weights from comma-separated string */
    void loadWeights(const std::string& str);

    /** @brief Convert Eigen vector to comma-separated string */
    static std::string vectorToString(const Eigen::VectorXf& vec);

    /** @brief Parse comma-separated string to Eigen vector */
    static Eigen::VectorXf stringToVector(const std::string& str);

private:
    Eigen::VectorXf weights;  ///< Weight vector (bias = weights[0])

    static Eigen::MatrixXf vectorToEigenMatrix(const std::vector<std::vector<float>>& X);
    static Eigen::VectorXf vectorToEigenVector(const std::vector<float>& v);
    static std::vector<float> eigenVectorToStd(const Eigen::VectorXf& v);
    static Eigen::MatrixXf addConstantColumn(const Eigen::MatrixXf& X);
};

/**
 * @brief Read CSV file: all columns except last -> data, last column -> labels
 * @param filename   CSV file path (first row is header, skipped)
 * @param data       Output: feature matrix [N samples][M features]
 * @param lastColumn Output: target values [N samples]
 */
void readCSV(const std::string& filename,
             std::vector<std::vector<float>>& data,
             std::vector<float>& lastColumn);

} // namespace rxs

#endif // RXS_LINEAR_REGRESSION_H