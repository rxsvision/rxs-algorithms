#include "linear_regression.h"
#include <iostream>
#include <cassert>

int main()
{
    // Training data: y = 2*x1 + 3*x2 + 1 (bias=1, w1=2, w2=3)
    std::vector<std::vector<float>> X = {
        {1.0f, 2.0f},
        {2.0f, 3.0f},
        {3.0f, 4.0f},
        {4.0f, 5.0f},
        {5.0f, 6.0f}
    };
    std::vector<float> y = {9.0f, 14.0f, 19.0f, 24.0f, 29.0f};

    // Fit
    rxs::LinearRegression lr;
    lr.fit(X, y);

    // Check weights (bias, w1, w2)
    auto w = lr.getWeights();
    std::cout << "Weights: ";
    for (size_t i = 0; i < w.size(); ++i) {
        std::cout << w[i];
        if (i < w.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    // Predict
    auto pred = lr.predict(X);
    std::cout << "Predictions: ";
    for (size_t i = 0; i < pred.size(); ++i) {
        std::cout << pred[i];
        if (i < pred.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    // Weight serialization round-trip
    std::string saved = lr.saveWeights();
    std::cout << "Saved weights: " << saved << std::endl;

    rxs::LinearRegression lr2;
    lr2.loadWeights(saved);
    auto w2 = lr2.getWeights();
    std::cout << "Loaded weights: ";
    for (size_t i = 0; i < w2.size(); ++i) {
        std::cout << w2[i];
        if (i < w2.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    return 0;
}