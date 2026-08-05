#include "ellipse_fitting.h"

#include <iostream>
#include <random>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

// ---------------------------------------------------------------------------
// Synthetic 2D ellipse generation with Gaussian noise
//   Ellipse: center=(cx,cy), semi-axes (a >= b), rotation theta
//   Point on ellipse at parameter t:
//     x = cx + cos(theta)*a*cos(t) - sin(theta)*b*sin(t)
//     y = cy + sin(theta)*a*cos(t) + cos(theta)*b*sin(t)
//   Noise: N(0, sigma^2) on each coordinate
// ---------------------------------------------------------------------------

struct TestCase {
    Eigen::Vector2f center;
    float a;
    float b;
    float theta;
    float sigma;
    int n_points;
};

static std::vector<Eigen::Vector2f> generateEllipsePoints(const TestCase& tc,
                                                           unsigned int seed = 42)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, tc.sigma);
    std::uniform_real_distribution<float> angle(0.0f, 2.0f * static_cast<float>(M_PI));

    std::vector<Eigen::Vector2f> pts;
    pts.reserve(tc.n_points);

    float ct = std::cos(tc.theta), st = std::sin(tc.theta);
    for (int i = 0; i < tc.n_points; ++i) {
        float t = angle(rng);
        float u = tc.a * std::cos(t);
        float v = tc.b * std::sin(t);
        // Rotate from local to world: (x,y) = R(-theta) * (u,v)
        float x = ct * u - st * v + tc.center.x() + noise(rng);
        float y = st * u + ct * v + tc.center.y() + noise(rng);
        pts.emplace_back(x, y);
    }
    return pts;
}

// ---------------------------------------------------------------------------
// Test: fit 2D ellipse and verify accuracy
// ---------------------------------------------------------------------------

static bool runTest(const TestCase& tc, const std::string& name)
{
    std::cout << "=== " << name << " ===" << std::endl;
    std::cout << "  True: center=(" << tc.center.x() << ", " << tc.center.y()
              << ") a=" << tc.a << " b=" << tc.b
              << " theta=" << tc.theta << " sigma=" << tc.sigma
              << " N=" << tc.n_points << std::endl;

    auto pts = generateEllipsePoints(tc);
    rxs::Ellipse2D fit = rxs::fitEllipse2D(pts, true, true, 0.95f);

    if (!fit.valid) {
        std::cout << "  FAIL: fitEllipse2D returned invalid result" << std::endl;
        return false;
    }

    float center_err = (fit.center - tc.center).norm();
    float a_err_pct  = std::abs(fit.a - tc.a) / tc.a * 100.0f;
    float b_err_pct  = std::abs(fit.b - tc.b) / tc.b * 100.0f;

    std::cout << "  Fit:  center=(" << fit.center.x() << ", " << fit.center.y()
              << ") a=" << fit.a << " b=" << fit.b
              << " theta=" << fit.theta << std::endl;
    std::cout << "  Error: center=" << center_err << "mm"
              << " a=" << a_err_pct << "%"
              << " b=" << b_err_pct << "%" << std::endl;

    // Verification criteria
    const float center_tol = 0.1f;  // mm
    const float axis_tol_pct = 5.0f; // %

    bool pass_center = center_err < center_tol;
    bool pass_a = a_err_pct < axis_tol_pct;
    bool pass_b = b_err_pct < axis_tol_pct;
    bool all_pass = pass_center && pass_a && pass_b;

    std::cout << "  center<" << center_tol << "mm: " << (pass_center ? "PASS" : "FAIL") << std::endl;
    std::cout << "  a<" << axis_tol_pct << "%: " << (pass_a ? "PASS" : "FAIL") << std::endl;
    std::cout << "  b<" << axis_tol_pct << "%: " << (pass_b ? "PASS" : "FAIL") << std::endl;
    std::cout << "  Result: " << (all_pass ? "PASS" : "FAIL") << std::endl;
    return all_pass;
}

int main()
{
    std::cout << "============================================================\n";
    std::cout << "rxs_ellipse_fitting unit tests\n";
    std::cout << "============================================================\n\n";

    // Test 1: Standard ellipse (a=2.0, b=1.5, theta=0.5, center=(1,2))
    TestCase tc1;
    tc1.center = Eigen::Vector2f(1.0f, 2.0f);
    tc1.a = 2.0f;
    tc1.b = 1.5f;
    tc1.theta = 0.5f;
    tc1.sigma = 0.02f;
    tc1.n_points = 200;
    bool r1 = runTest(tc1, "Test 1: standard ellipse (a=2.0, b=1.5, theta=0.5)");

    // Test 2: Near-circle ellipse (a=2.0, b=1.95, theta=0.3) — near-circle constraint should fire
    TestCase tc2;
    tc2.center = Eigen::Vector2f(0.5f, -1.0f);
    tc2.a = 2.0f;
    tc2.b = 1.95f;
    tc2.theta = 0.3f;
    tc2.sigma = 0.02f;
    tc2.n_points = 200;
    bool r2 = runTest(tc2, "Test 2: near-circle ellipse (a=2.0, b=1.95)");

    // Test 3: Axis-aligned ellipse (theta=0)
    TestCase tc3;
    tc3.center = Eigen::Vector2f(-3.0f, 4.0f);
    tc3.a = 3.0f;
    tc3.b = 1.0f;
    tc3.theta = 0.0f;
    tc3.sigma = 0.02f;
    tc3.n_points = 200;
    bool r3 = runTest(tc3, "Test 3: axis-aligned ellipse (a=3.0, b=1.0, theta=0)");

    // Test 4: Mode switching API verification
    std::cout << "\n============================================================\n";
    std::cout << "Test 4: Mode switching API (MODE_D / MODE_C)\n";
    std::cout << "============================================================\n";
    rxs::EllipseParams params_d, params_c;
    params_d.mode = rxs::EllipseFitMode::MODE_D;
    params_c.mode = rxs::EllipseFitMode::MODE_C;
    params_c.pcl_leaf_size = 0.05f;
    params_c.pcl_max_iter = 10000;
    std::cout << "  MODE_D: lm_algebraic=" << params_d.enable_lm_algebraic << ", lm_sampson=" << params_d.enable_lm_sampson << "\n";
    std::cout << "  MODE_C: pcl_leaf=" << params_c.pcl_leaf_size << ", pcl_max_iter=" << params_c.pcl_max_iter << "\n";
    rxs::CP empty_cloud(new rxs::CloudT);
    rxs::EllipseResult res_d = rxs::fitEllipsesOnPlane(empty_cloud, params_d);
    rxs::EllipseResult res_c = rxs::fitEllipsesOnPlane(empty_cloud, params_c);
    bool r4 = (!res_d.valid && !res_c.valid && !res_d.error.empty() && !res_c.error.empty());
    std::cout << "  MODE_D empty: valid=" << res_d.valid << ", error=\"" << res_d.error << "\"\n";
    std::cout << "  MODE_C empty: valid=" << res_c.valid << ", error=\"" << res_c.error << "\"\n";
    std::cout << "Test 4: " << (r4 ? "PASS" : "FAIL") << "\n";
    
    // Summary
    std::cout << "\n============================================================\n";
    std::cout << "Summary\n";
    std::cout << "============================================================\n";
    std::cout << "Test 1: " << (r1 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 2: " << (r2 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 3: " << (r3 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 4: " << (r4 ? "PASS" : "FAIL") << "\n";

    return (r1 && r2 && r3 && r4) ? 0 : 1;
}
