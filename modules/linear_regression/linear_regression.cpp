#include "linear_regression.h"
#include <fstream>
#include <sstream>

namespace rxs {

// ---------------------------------------------------------------------------
// Core: fit via normal equation w = (X^T X)^(-1) X^T y (LDLT decomposition)
// ---------------------------------------------------------------------------
void LinearRegression::fit(const std::vector<std::vector<float>>& X,
                           const std::vector<float>& y)
{
    Eigen::MatrixXf X_eigen = addConstantColumn(vectorToEigenMatrix(X));
    Eigen::VectorXf y_eigen = vectorToEigenVector(y);
    weights = (X_eigen.transpose() * X_eigen).ldlt().solve(X_eigen.transpose() * y_eigen);
}

// ---------------------------------------------------------------------------
// Predict
// ---------------------------------------------------------------------------
std::vector<float> LinearRegression::predict(const std::vector<std::vector<float>>& X)
{
    Eigen::MatrixXf X_eigen = addConstantColumn(vectorToEigenMatrix(X));
    Eigen::VectorXf predictions = X_eigen * weights;
    return eigenVectorToStd(predictions);
}

std::vector<float> LinearRegression::getWeights() const
{
    return eigenVectorToStd(weights);
}

// ---------------------------------------------------------------------------
// Weight serialization
// ---------------------------------------------------------------------------
std::string LinearRegression::saveWeights()
{
    return vectorToString(weights);
}

void LinearRegression::loadWeights(const std::string& str)
{
    weights = stringToVector(str);
}

std::string LinearRegression::vectorToString(const Eigen::VectorXf& vec)
{
    std::ostringstream oss;
    for (int i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) oss << ",";
    }
    return oss.str();
}

Eigen::VectorXf LinearRegression::stringToVector(const std::string& str)
{
    std::istringstream iss(str);
    std::vector<float> values;
    std::string token;
    while (std::getline(iss, token, ',')) {
        values.push_back(std::stof(token));
    }
    return Eigen::Map<Eigen::VectorXf>(values.data(), static_cast<int>(values.size()));
}

// ---------------------------------------------------------------------------
// Type conversion helpers
// ---------------------------------------------------------------------------
Eigen::MatrixXf LinearRegression::vectorToEigenMatrix(const std::vector<std::vector<float>>& X)
{
    int rows = static_cast<int>(X.size());
    int cols = static_cast<int>(X[0].size());
    Eigen::MatrixXf mat(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = X[i][j];
        }
    }
    return mat;
}

Eigen::VectorXf LinearRegression::vectorToEigenVector(const std::vector<float>& v)
{
    int size = static_cast<int>(v.size());
    Eigen::VectorXf vec(size);
    for (int i = 0; i < size; ++i) {
        vec(i) = v[i];
    }
    return vec;
}

std::vector<float> LinearRegression::eigenVectorToStd(const Eigen::VectorXf& v)
{
    std::vector<float> result(v.size());
    for (int i = 0; i < v.size(); ++i) {
        result[i] = v(i);
    }
    return result;
}

Eigen::MatrixXf LinearRegression::addConstantColumn(const Eigen::MatrixXf& X)
{
    int rows = static_cast<int>(X.rows());
    Eigen::MatrixXf X_with_bias(rows, X.cols() + 1);
    X_with_bias << Eigen::MatrixXf::Ones(rows, 1), X;
    return X_with_bias;
}

// ---------------------------------------------------------------------------
// CSV reader: first row = header (skipped), last column = target
// ---------------------------------------------------------------------------
void readCSV(const std::string& filename,
             std::vector<std::vector<float>>& data,
             std::vector<float>& lastColumn)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return;
    }

    std::string line;
    bool firstLine = true;
    while (std::getline(file, line)) {
        if (firstLine) {
            firstLine = false;
            continue;
        }
        std::stringstream ss(line);
        std::string cell;
        std::vector<float> row;
        while (std::getline(ss, cell, ',')) {
            try {
                row.push_back(std::stof(cell));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Conversion error: " << e.what() << std::endl;
                return;
            }
        }
        if (row.empty()) continue;
        lastColumn.push_back(row.back());
        row.pop_back();
        data.push_back(row);
    }
    file.close();
}

} // namespace rxs