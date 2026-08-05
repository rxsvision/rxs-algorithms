#include "ellipse_fitting.h"

// PCL
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// Eigen
#include <Eigen/Eigenvalues>

// STL
#include <random>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <string>

namespace rxs {

// ---------------------------------------------------------------------------
// Timer (P0-4): lightweight RAII-style high-resolution clock
// ---------------------------------------------------------------------------
class Timer {
public:
    using clock = std::chrono::high_resolution_clock;
    void start() { start_ = clock::now(); }
    double elapsedMs() const {
        return std::chrono::duration<double, std::milli>(clock::now() - start_).count();
    }
private:
    clock::time_point start_;
};

// ---------------------------------------------------------------------------
// Plane frame utilities (from benchmark lines 146-201)
// ---------------------------------------------------------------------------

/// Build plane local frame: U, V are two orthonormal in-plane vectors
static void buildPlaneFrame(const Eigen::Vector3f& normal,
                            Eigen::Vector3f& U, Eigen::Vector3f& V)
{
    // Pick the world axis least aligned with normal for cross product
    Eigen::Vector3f ref = (std::abs(normal.x()) < 0.9f)
                          ? Eigen::Vector3f::UnitX()
                          : Eigen::Vector3f::UnitY();
    U = normal.cross(ref).normalized();
    V = normal.cross(U).normalized();
}

/// Project 3D point cloud to 2D plane coordinates (u, v)
static void projectToPlane2D(const CP& cloud,
                             const Eigen::Vector3f& centroid,
                             const Eigen::Vector3f& normal,
                             const Eigen::Vector3f& U,
                             const Eigen::Vector3f& V,
                             std::vector<Eigen::Vector2f>& out2d)
{
    out2d.resize(cloud->size());
    Eigen::Vector3f n = normal.normalized();
    for (size_t i = 0; i < cloud->size(); ++i) {
        Eigen::Vector3f p = (*cloud)[i].getVector3fMap() - centroid;
        // Project onto plane (remove normal component)
        p = p - p.dot(n) * n;
        out2d[i] = Eigen::Vector2f(p.dot(U), p.dot(V));
    }
}

/// Lift 2D ellipse center back to 3D
static Eigen::Vector3f lift2DCenterTo3D(const Eigen::Vector2f& c2d,
                                        const Eigen::Vector3f& centroid,
                                        const Eigen::Vector3f& normal,
                                        const Eigen::Vector3f& U,
                                        const Eigen::Vector3f& V)
{
    Eigen::Vector3f n = normal.normalized();
    // Map center to 3D, then re-project to ensure it lies on the fitted plane
    Eigen::Vector3f c3d = centroid + c2d.x() * U + c2d.y() * V;
    Eigen::Vector3f toCentroid = c3d - centroid;
    toCentroid = toCentroid - toCentroid.dot(n) * n;
    return centroid + toCentroid;
}

/// Compute 2D planar distance between two 3D points (for evaluation)
static float planarDistance2D(const Eigen::Vector3f& p1,
                              const Eigen::Vector3f& p2,
                              const Eigen::Vector3f& centroid,
                              const Eigen::Vector3f& normal,
                              const Eigen::Vector3f& U,
                              const Eigen::Vector3f& V)
{
    Eigen::Vector3f n = normal.normalized();
    Eigen::Vector3f d1 = (p1 - centroid); d1 = d1 - d1.dot(n) * n;
    Eigen::Vector3f d2 = (p2 - centroid); d2 = d2 - d2.dot(n) * n;
    Eigen::Vector2f q1(d1.dot(U), d1.dot(V));
    Eigen::Vector2f q2(d2.dot(U), d2.dot(V));
    return (q1 - q2).norm();
}

// ---------------------------------------------------------------------------
// Ellipse coefficient extraction (from benchmark lines 347-404)
// ---------------------------------------------------------------------------

/// Extract ellipse center from 6 coefficients
/// Equation: A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
/// Uses Fitzgibbon's notation: a=A, b=B/2, c=C, d=D/2, f=E/2
/// Center: h = (2*C*D - B*E) / (B^2 - 4*A*C), k = (2*A*E - B*D) / (B^2 - 4*A*C)
static bool ellipseCenterFromCoeffs(const Eigen::Matrix<float, 6, 1>& c,
                                    Eigen::Vector2f& center)
{
    float A = c(0), B = c(1), Cc = c(2), D = c(3), E = c(4);
    float denom2 = B * B - 4.0f * A * Cc;
    if (std::abs(denom2) < 1e-12f) return false;
    center.x() = (2.0f * Cc * D - B * E) / denom2;
    center.y() = (2.0f * A * E - B * D) / denom2;
    return true;
}

/// Extract ellipse geometry (center, a, b, theta) from 6 coefficients
/// Returns a >= b
static bool ellipseGeometryFromCoeffs(const Eigen::Matrix<float, 6, 1>& c,
                                      Eigen::Vector2f& center,
                                      float& a, float& b, float& theta)
{
    float A = c(0), B = c(1), Cc = c(2), D = c(3), E = c(4), F = c(5);

    float denom = B * B - 4.0f * A * Cc;
    if (std::abs(denom) < 1e-12f) return false;
    center.x() = (2.0f * Cc * D - B * E) / denom;
    center.y() = (2.0f * A * E - B * D) / denom;

    // Translate to center: F' = F + A*h^2 + B*h*k + C*k^2 + D*h + E*k
    float h = center.x(), k = center.y();
    float Fp = F + A * h * h + B * h * k + Cc * k * k + D * h + E * k;
    if (Fp >= 0) return false;  // Ellipse requires F' < 0

    // Rotation angle (eliminate cross term)
    theta = 0.5f * std::atan2(B, A - Cc);
    float ct = std::cos(theta), st = std::sin(theta);
    float Ap = A * ct * ct + B * ct * st + Cc * st * st;
    float Cp = A * st * st - B * ct * st + Cc * ct * ct;

    if (Ap <= 0 || Cp <= 0) return false;
    a = std::sqrt(-Fp / Ap);
    b = std::sqrt(-Fp / Cp);

    // Ensure a >= b
    if (a < b) {
        std::swap(a, b);
        theta += static_cast<float>(M_PI_2);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Taubin AMS algebraic ellipse fitting (from benchmark lines 262-341)
// ---------------------------------------------------------------------------

bool fitEllipseTaubin(const std::vector<Eigen::Vector2f>& pts,
                      Eigen::Matrix<float, 6, 1>& coeffs)
{
    if (pts.size() < 5) return false;
    const int N = static_cast<int>(pts.size());

    // Centering
    Eigen::Vector2f mu = Eigen::Vector2f::Zero();
    for (const auto& p : pts) mu += p;
    mu /= float(N);

    // Accumulate moments
    float X2 = 0, Y2 = 0, XY = 0, X = 0, Y = 0;
    float X3 = 0, Y3 = 0, X2Y = 0, XY2 = 0;
    float X4 = 0, Y4 = 0, X3Y = 0, X2Y2 = 0, XY3 = 0;
    for (const auto& p : pts) {
        float x = p.x() - mu.x(), y = p.y() - mu.y();
        float x2 = x * x, y2 = y * y, xy = x * y;
        X += x; Y += y;
        X2 += x2; Y2 += y2; XY += xy;
        X3 += x2 * x; Y3 += y2 * y;
        X2Y += x2 * y; XY2 += x * y2;
        X4 += x2 * x2; Y4 += y2 * y2;
        X3Y += x2 * xy; X2Y2 += x2 * y2; XY3 += x * y2 * y;
    }
    float n = float(N);

    // Taubin's method: construct matrices and solve generalized eigenvalue problem
    Eigen::Matrix3f M = (Eigen::Matrix3f() <<
        X4,       X2Y,      X3,
        X2Y,      X2Y2,     XY2,
        X3,       XY2,      X2).finished() / n;

    Eigen::Matrix3f S = (Eigen::Matrix3f() <<
        X2Y2 + Y4, X3Y + XY3, X2Y + Y3,
        X3Y + XY3, X4 + X2Y2, X3 + XY2,
        X2Y + Y3, X3 + XY2, X2 + Y2).finished() / n;

    // Solve M * v = lambda * S * v, find smallest eigenvalue
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix3f> solver;
    solver.compute(M, S);
    if (solver.info() != Eigen::Success) return false;

    Eigen::Vector3f v = solver.eigenvectors().col(0); // Smallest eigenvalue
    float A = v(0), B = v(1), Cc = v(2);

    // Solve for D, E via normal equations (first-order conditions)
    Eigen::Vector2f rhs;
    rhs << -(A * X3 + B * X2Y + Cc * XY2),
           -(A * X2Y + B * XY2 + Cc * Y3);
    Eigen::Matrix2f Mt;
    Mt << X2, XY, XY, Y2;
    if (std::abs(Mt.determinant()) < 1e-12f) return false;
    Eigen::Vector2f de = Mt.ldlt().solve(rhs);
    float Dd = de(0), E = de(1);
    float F = -(A * X2 + B * XY + Cc * Y2 + Dd * X + E * Y) / n;

    // Centering compensation: original coords = centered coords + mu
    float mx = mu.x(), my = mu.y();
    float Dp = Dd - 2.0f * A * mx - B * my;
    float Ep = E - B * mx - 2.0f * Cc * my;
    float Fp = F - Dd * mx - E * my + A * mx * mx + B * mx * my + Cc * my * my;
    coeffs << A, B, Cc, Dp, Ep, Fp;
    return true;
}

// ---------------------------------------------------------------------------
// LM refinement: algebraic residual with analytic Jacobian (benchmark 410-506)
// ---------------------------------------------------------------------------

bool refineEllipseLM_algebraic(const std::vector<Eigen::Vector2f>& pts,
                               Eigen::Vector2f& center,
                               float& a, float& b, float& theta,
                               int max_iter,
                               float cost_tol)
{
    if (pts.size() < 5 || a <= 0 || b <= 0) return false;

    Eigen::VectorXf p(5);
    p << center.x(), center.y(), a, b, theta;

    auto computeResidual = [&](const Eigen::VectorXf& params, Eigen::VectorXf& r) {
        float cx = params(0), cy = params(1), aa = params(2), bb = params(3), th = params(4);
        float ct = std::cos(th), st = std::sin(th);
        r.resize(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            float x = pts[i].x() - cx;
            float y = pts[i].y() - cy;
            float u = ct * x + st * y;
            float v = -st * x + ct * y;
            r(i) = u * u / (aa * aa) + v * v / (bb * bb) - 1.0f;
        }
    };

    Eigen::VectorXf r;
    computeResidual(p, r);
    float cost = 0.5f * r.squaredNorm();

    float lambda = 1e-3f;

    for (int iter = 0; iter < max_iter; ++iter) {
        float cx = p(0), cy = p(1), aa = p(2), bb = p(3), th = p(4);
        float ct = std::cos(th), st = std::sin(th);

        int N = static_cast<int>(pts.size());
        Eigen::MatrixXf J(N, 5);

        for (int i = 0; i < N; ++i) {
            float x = pts[i].x() - cx;
            float y = pts[i].y() - cy;
            float u = ct * x + st * y;
            float v = -st * x + ct * y;

            // Partial derivatives (analytic Jacobian)
            // du/dcx=-ct, du/dcy=-st, du/dth=v
            // dv/dcx= st, dv/dcy=-ct, dv/dth=-u
            // r = u^2/a^2 + v^2/b^2 - 1
            J(i, 0) = -2.0f * u * ct / (aa * aa) + 2.0f * v * st / (bb * bb);
            J(i, 1) = -2.0f * u * st / (aa * aa) - 2.0f * v * ct / (bb * bb);
            J(i, 2) = -2.0f * u * u / (aa * aa * aa);
            J(i, 3) = -2.0f * v * v / (bb * bb * bb);
            J(i, 4) = 2.0f * u * v / (aa * aa) - 2.0f * u * v / (bb * bb);
        }

        Eigen::MatrixXf JtJ = J.transpose() * J;
        Eigen::VectorXf Jtr = J.transpose() * r;

        // Damped normal equation: (JtJ + lambda*I) dp = -Jtr
        Eigen::MatrixXf A_mat = JtJ;
        for (int k = 0; k < 5; ++k) A_mat(k, k) *= (1.0f + lambda);

        Eigen::VectorXf dp = A_mat.ldlt().solve(-Jtr);
        if (!std::isfinite(dp(0))) break;

        Eigen::VectorXf p_new = p + dp;
        // Constraint: a, b > 0
        if (p_new(2) <= 0 || p_new(3) <= 0) {
            lambda *= 10.0f;
            continue;
        }

        Eigen::VectorXf r_new;
        computeResidual(p_new, r_new);
        float cost_new = 0.5f * r_new.squaredNorm();

        if (cost_new < cost) {
            float cost_drop = cost - cost_new;
            p = p_new;
            r = r_new;
            cost = cost_new;
            lambda *= 0.5f;
            if (cost_drop < cost_tol) break;  // Converged
        } else {
            lambda *= 2.0f;
            if (lambda > 1e8f) break;
        }
    }

    center = Eigen::Vector2f(p(0), p(1));
    a = p(2);
    b = p(3);
    theta = p(4);
    return true;
}

// ---------------------------------------------------------------------------
// LM refinement: Sampson distance with numerical Jacobian (benchmark 516-603)
// ---------------------------------------------------------------------------

bool refineEllipseLM_sampson(const std::vector<Eigen::Vector2f>& pts,
                             Eigen::Vector2f& center,
                             float& a, float& b, float& theta,
                             int max_iter,
                             float cost_tol)
{
    if (pts.size() < 5 || a <= 0 || b <= 0) return false;

    Eigen::VectorXf p(5);
    p << center.x(), center.y(), a, b, theta;

    auto computeResidual = [&](const Eigen::VectorXf& params, Eigen::VectorXf& r) {
        float cx = params(0), cy = params(1), aa = params(2), bb = params(3), th = params(4);
        float ct = std::cos(th), st = std::sin(th);
        float a2 = aa * aa, b2 = bb * bb;
        float a4 = a2 * a2, b4 = b2 * b2;
        r.resize(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            float x = pts[i].x() - cx;
            float y = pts[i].y() - cy;
            float u = ct * x + st * y;
            float v = -st * x + ct * y;
            float F = u * u / a2 + v * v / b2 - 1.0f;
            float grad_norm = 2.0f * std::sqrt(u * u / a4 + v * v / b4);
            // Degradation protection: fall back to algebraic residual when gradient is too small
            r(i) = (grad_norm > 1e-10f) ? (F / grad_norm) : F;
        }
    };

    Eigen::VectorXf r;
    computeResidual(p, r);
    float cost = 0.5f * r.squaredNorm();

    float lambda = 1e-3f;
    const float h = 1e-5f;

    for (int iter = 0; iter < max_iter; ++iter) {
        int N = static_cast<int>(pts.size());
        Eigen::MatrixXf J(N, 5);

        // Numerical Jacobian (forward differences)
        for (int k = 0; k < 5; ++k) {
            Eigen::VectorXf p_plus = p;
            p_plus(k) += h;
            Eigen::VectorXf r_plus;
            computeResidual(p_plus, r_plus);
            J.col(k) = (r_plus - r) / h;
        }

        Eigen::MatrixXf JtJ = J.transpose() * J;
        Eigen::VectorXf Jtr = J.transpose() * r;

        // Damped normal equation: (JtJ + lambda*I) dp = -Jtr
        Eigen::MatrixXf A_mat = JtJ;
        for (int k = 0; k < 5; ++k) A_mat(k, k) *= (1.0f + lambda);

        Eigen::VectorXf dp = A_mat.ldlt().solve(-Jtr);
        if (!std::isfinite(dp(0))) break;

        Eigen::VectorXf p_new = p + dp;
        // Constraint: a, b > 0
        if (p_new(2) <= 0 || p_new(3) <= 0) {
            lambda *= 10.0f;
            continue;
        }

        Eigen::VectorXf r_new;
        computeResidual(p_new, r_new);
        float cost_new = 0.5f * r_new.squaredNorm();

        if (cost_new < cost) {
            float cost_drop = cost - cost_new;
            p = p_new;
            r = r_new;
            cost = cost_new;
            lambda *= 0.5f;
            if (cost_drop < cost_tol) break;  // Converged
        } else {
            lambda *= 2.0f;
            if (lambda > 1e8f) break;
        }
    }

    center = Eigen::Vector2f(p(0), p(1));
    a = p(2);
    b = p(3);
    theta = p(4);
    return true;
}

// ---------------------------------------------------------------------------
// Circle RANSAC (from benchmark lines 1010-1072, extracted from lambda)
// ---------------------------------------------------------------------------

/// Fit circle from 3 points (analytic)
static bool fitCircle3Points(const Eigen::Vector2f& p1,
                             const Eigen::Vector2f& p2,
                             const Eigen::Vector2f& p3,
                             Eigen::Vector2f& center, float& radius)
{
    float d = 2.0f * (p1.x() * (p2.y() - p3.y()) +
                      p2.x() * (p3.y() - p1.y()) +
                      p3.x() * (p1.y() - p2.y()));
    if (std::abs(d) < 1e-10f) return false;
    float ux = ((p1.x() * p1.x() + p1.y() * p1.y()) * (p2.y() - p3.y()) +
                (p2.x() * p2.x() + p2.y() * p2.y()) * (p3.y() - p1.y()) +
                (p3.x() * p3.x() + p3.y() * p3.y()) * (p1.y() - p2.y())) / d;
    float uy = ((p1.x() * p1.x() + p1.y() * p1.y()) * (p3.x() - p2.x()) +
                (p2.x() * p2.x() + p2.y() * p2.y()) * (p1.x() - p3.x()) +
                (p3.x() * p3.x() + p3.y() * p3.y()) * (p2.x() - p1.x())) / d;
    center = Eigen::Vector2f(ux, uy);
    radius = (p1 - center).norm();
    return true;
}

/// 2D circle RANSAC: 3-point sampling + inlier counting
static bool circleRANSAC(std::vector<Eigen::Vector2f>& points,
                         Eigen::Vector2f& best_center,
                         float& best_radius,
                         std::vector<int>& best_inliers,
                         unsigned int seed,
                         int max_iter,
                         float dist_threshold,
                         float r_min, float r_max)
{
    int N = static_cast<int>(points.size());
    if (N < 3) return false;
    std::mt19937 rng(seed == 0 ? std::random_device{}() : seed);
    int best_count = 0;
    best_center = Eigen::Vector2f::Zero();
    best_radius = 0;

    for (int iter = 0; iter < max_iter; ++iter) {
        int i1 = rng() % N, i2 = rng() % N, i3 = rng() % N;
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;
        Eigen::Vector2f c;
        float r;
        if (!fitCircle3Points(points[i1], points[i2], points[i3], c, r)) continue;
        if (r < r_min || r > r_max) continue;

        int count = 0;
        for (int i = 0; i < N; ++i) {
            if (std::abs((points[i] - c).norm() - r) < dist_threshold) count++;
        }
        if (count > best_count) {
            best_count = count;
            best_center = c;
            best_radius = r;
        }
    }

    if (best_count < 10) return false;

    // Collect inliers
    best_inliers.clear();
    for (int i = 0; i < N; ++i) {
        if (std::abs((points[i] - best_center).norm() - best_radius) < dist_threshold) {
            best_inliers.push_back(i);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Circle segmentation pipeline (from benchmark segmentCircles2D, lines 928-1116)
//   P0-1: plane_max_iter now parameterized (caller decides via fast_mode)
// ---------------------------------------------------------------------------

/// Internal segmentation result
struct SegmentationResult {
    Eigen::Vector3f fitted_normal;
    Eigen::Vector3f fitted_centroid;
    Eigen::Vector3f U, V;                                  // Plane local frame
    std::vector<std::vector<Eigen::Vector2f>> clusters_2d; // 2D projected points per cluster
    std::vector<Eigen::Vector2f> circle_centers_2d;        // Circle RANSAC centers (LM init)
    std::vector<float> circle_radii;                       // Circle RANSAC radii (LM init)
    EllipseError error_code = EllipseError::OK;            // P0-1: structured error
    bool success;
};

/// RANSAC plane + non-plane extraction + voxel + 2D projection + circle RANSAC
/// @param plane_max_iter  Plane RANSAC max iterations (fast_mode: 2000, default: 5000)
static SegmentationResult segmentCircles2D(const CP& cloud,
                                           const EllipseParams& params,
                                           int plane_max_iter)
{
    SegmentationResult result;
    result.success = false;
    if (cloud->empty()) {
        result.error_code = EllipseError::EMPTY_CLOUD;
        return result;
    }

    // Step 1: RANSAC plane segmentation
    const float plane_dist_threshold = 0.05f;
    pcl::SampleConsensusModelPlane<PointT>::Ptr plane_model(
        new pcl::SampleConsensusModelPlane<PointT>(cloud));
    pcl::RandomSampleConsensus<PointT> ransac(plane_model, plane_dist_threshold);
    ransac.setMaxIterations(plane_max_iter);
    if (!ransac.computeModel()) {
        result.error_code = EllipseError::PLANE_SEGMENT_FAIL;
        return result;
    }

    Eigen::VectorXf plane_coeffs;
    ransac.getModelCoefficients(plane_coeffs);
    result.fitted_normal = Eigen::Vector3f(plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]).normalized();

    pcl::Indices inliers;
    ransac.getInliers(inliers);

    Eigen::Vector3f centroid(0, 0, 0);
    for (int idx : inliers) centroid += (*cloud)[idx].getVector3fMap();
    centroid /= float(inliers.size());
    result.fitted_centroid = centroid;

    // Step 2: Extract non-plane points (ellipse/circle features)
    CP non_plane(new CloudT);
    for (const auto& pt : cloud->points) {
        float dist = std::abs(pt.x * plane_coeffs[0] + pt.y * plane_coeffs[1]
                              + pt.z * plane_coeffs[2] + plane_coeffs[3]);
        if (dist > params.height_threshold) {
            non_plane->push_back(pt);
        }
    }
    non_plane->width = non_plane->size();
    non_plane->height = 1;

    if (non_plane->empty()) {
        result.error_code = EllipseError::NO_NON_PLANE_POINTS;
        return result;
    }

    // Step 3: Plane local frame
    buildPlaneFrame(result.fitted_normal, result.U, result.V);

    // Step 4: Voxel downsampling
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(non_plane);
    voxel.setLeafSize(params.voxel_leaf, params.voxel_leaf, params.voxel_leaf);
    CP non_plane_ds(new CloudT);
    voxel.filter(*non_plane_ds);

    if (non_plane_ds->empty()) {
        result.error_code = EllipseError::NO_NON_PLANE_POINTS;
        return result;
    }

    // Step 5: Project to 2D
    Eigen::Vector3f n = result.fitted_normal;
    std::vector<Eigen::Vector2f> pts2d;
    pts2d.reserve(non_plane_ds->size());
    for (const auto& pt : non_plane_ds->points) {
        Eigen::Vector3f p = pt.getVector3fMap() - centroid;
        p = p - p.dot(n) * n;
        pts2d.emplace_back(p.dot(result.U), p.dot(result.V));
    }

    // Step 6: Circle RANSAC -- extract 2 circles sequentially
    std::vector<Eigen::Vector2f> remaining = pts2d;
    for (int circle_idx = 0; circle_idx < 2; ++circle_idx) {
        Eigen::Vector2f center;
        float radius;
        std::vector<int> inliers;
        if (!circleRANSAC(remaining, center, radius, inliers,
                          params.seed, params.circle_max_iter,
                          params.circle_dist_threshold,
                          params.circle_r_min, params.circle_r_max)) {
            break;
        }

        // Extract inlier points
        std::vector<Eigen::Vector2f> cluster_pts;
        cluster_pts.reserve(inliers.size());
        for (int idx : inliers) {
            cluster_pts.push_back(remaining[idx]);
        }
        if (cluster_pts.size() >= 10) {
            result.clusters_2d.push_back(std::move(cluster_pts));
            // Save circle RANSAC center/radius as LM initial values
            result.circle_centers_2d.push_back(center);
            result.circle_radii.push_back(radius);
        }

        // Remove inliers
        std::vector<Eigen::Vector2f> next_pts;
        std::vector<bool> is_inlier(remaining.size(), false);
        for (int idx : inliers) is_inlier[idx] = true;
        for (size_t i = 0; i < remaining.size(); ++i) {
            if (!is_inlier[i]) next_pts.push_back(remaining[i]);
        }
        remaining = std::move(next_pts);
        if (remaining.empty()) break;
    }

    if (result.clusters_2d.size() == 2) {
        result.success = true;
    } else {
        result.error_code = EllipseError::CLUSTER_FAIL;
    }
    return result;
}

// ---------------------------------------------------------------------------
// Internal: fit 2D ellipse with optional circle RANSAC initialization
//   (from benchmark runAlgoB use_taubin=true logic, lines 1159-1231)
// ---------------------------------------------------------------------------

static Ellipse2D fitEllipse2DWithInit(const std::vector<Eigen::Vector2f>& pts,
                                      const Eigen::Vector2f* circle_center,
                                      float circle_radius,
                                      bool enable_lm_algebraic,
                                      bool enable_lm_sampson,
                                      float near_circle_ratio)
{
    Ellipse2D result;
    if (pts.size() < 10) return result;

    // Step 1: Taubin AMS algebraic fit
    Eigen::Matrix<float, 6, 1> coeffs;
    if (!fitEllipseTaubin(pts, coeffs)) return result;

    // Step 2: Determine LM initial values
    //   Priority 1: circle RANSAC result (accurate center + radius)
    //   Priority 2: Taubin geometry parameters
    //   Priority 3: centroid + mean radius (fallback)
    Eigen::Vector2f init_center;
    float init_a = 0, init_b = 0, init_theta = 0;

    if (circle_center != nullptr && circle_radius > 0) {
        // Priority 1: circle RANSAC
        init_center = *circle_center;
        init_a = circle_radius;
        init_b = circle_radius;  // Near-circle assumption (a/b ~ 1)
        init_theta = 0.0f;
    } else {
        // Priority 2: Taubin geometry
        bool geom_ok = ellipseGeometryFromCoeffs(coeffs, init_center, init_a, init_b, init_theta);
        if (!geom_ok) {
            // Priority 3: centroid + mean radius
            init_center = Eigen::Vector2f::Zero();
            for (const auto& p : pts) init_center += p;
            init_center /= float(pts.size());
            float mean_r = 0;
            for (const auto& p : pts) mean_r += (p - init_center).norm();
            mean_r /= float(pts.size());
            init_a = mean_r;
            init_b = mean_r;
            init_theta = 0.0f;
        }
    }

    // Step 3: Two-step LM
    //   algebraic first (fast convergence), then sampson (geometric refinement)
    bool alg_ok = false, samp_ok = false;
    if (enable_lm_algebraic) {
        alg_ok = refineEllipseLM_algebraic(pts, init_center, init_a, init_b, init_theta);
    }
    if (enable_lm_sampson) {
        samp_ok = refineEllipseLM_sampson(pts, init_center, init_a, init_b, init_theta);
    }
    bool lm_ok = enable_lm_sampson ? samp_ok : alg_ok;

    if (!lm_ok) {
        // Fallback: extract center from Taubin coefficients
        Eigen::Vector2f fallback_center;
        if (!ellipseCenterFromCoeffs(coeffs, fallback_center)) return result;
        init_center = fallback_center;
    }

    // Step 4: Near-circle constraint -- force theta=0 when a/b > threshold
    if (init_a > 0 && init_b > 0) {
        float ratio = std::min(init_a, init_b) / std::max(init_a, init_b);
        if (ratio > near_circle_ratio) {
            init_theta = 0.0f;
        }
    }

    result.center = init_center;
    result.a = init_a;
    result.b = init_b;
    result.theta = init_theta;
    result.valid = true;
    return result;
}

// ---------------------------------------------------------------------------
// Public API: 2D-only ellipse fitting
// ---------------------------------------------------------------------------

Ellipse2D fitEllipse2D(const std::vector<Eigen::Vector2f>& pts,
                       bool enable_lm_algebraic,
                       bool enable_lm_sampson,
                       float near_circle_ratio)
{
    // No circle RANSAC initial values available in 2D-only mode
    return fitEllipse2DWithInit(pts, nullptr, 0.0f,
                                 enable_lm_algebraic, enable_lm_sampson,
                                 near_circle_ratio);
}

// ===========================================================================
// P0-2: ROI crop (built-in, based on CAD theoretical model)
// ===========================================================================

/// Crop cloud by merged AABB from roi_centers + roi_half_sizes, with z margin.
/// @return Cropped cloud; empty cloud on parameter mismatch (caller checks).
static CP cropROI(const CP& cloud, const EllipseParams& params)
{
    CP empty(new CloudT);
    if (params.roi_centers.empty()) return empty;
    if (params.roi_half_sizes.size() != params.roi_centers.size()) return empty;

    // Merge AABB over all ROI centers
    float xmin = std::numeric_limits<float>::max();
    float xmax = std::numeric_limits<float>::lowest();
    float ymin = xmin, ymax = xmax;
    float zmin = xmin, zmax = xmax;
    for (size_t i = 0; i < params.roi_centers.size(); ++i) {
        const auto& c = params.roi_centers[i];
        float half = params.roi_half_sizes[i];
        xmin = std::min(xmin, c.x() - half);
        xmax = std::max(xmax, c.x() + half);
        ymin = std::min(ymin, c.y() - half);
        ymax = std::max(ymax, c.y() + half);
        zmin = std::min(zmin, c.z() - params.roi_z_margin);
        zmax = std::max(zmax, c.z() + params.roi_z_margin);
    }

    CP cropped(new CloudT);
    cropped->reserve(cloud->size());
    for (const auto& p : *cloud) {
        if (p.x >= xmin && p.x <= xmax &&
            p.y >= ymin && p.y <= ymax &&
            p.z >= zmin && p.z <= zmax) {
            cropped->push_back(p);
        }
    }
    cropped->width = static_cast<uint32_t>(cropped->size());
    cropped->height = 1;
    cropped->is_dense = false;
    return cropped;
}

// ===========================================================================
// P0-3: MODE_D — RANSAC plane + Taubin AMS + two-step LM (fast)
//   fast_mode preset: plane_max_iter=2000, voxel_leaf=0.03, circle_max_iter=2000
//   Target: <50ms with ROI
// ===========================================================================

EllipseResult fitEllipsesOnPlane_D(CP cloud, const EllipseParams& params)
{
    EllipseResult result;
    result.valid = false;
    result.plane_normal = Eigen::Vector3f::Zero();
    result.plane_centroid = Eigen::Vector3f::Zero();
    result.error_code = EllipseError::OK;

    Timer timer;
    timer.start();

    if (!cloud || cloud->empty()) {
        result.error = "Input cloud is empty";
        result.error_code = EllipseError::EMPTY_CLOUD;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // fast_mode preset override (P0-3)
    // plane_max_iter: 5000 → 2000, voxel_leaf: 0.02 → 0.03, circle_max_iter: 50000 → 2000
    int plane_max_iter = params.fast_mode ? 2000 : 5000;
    EllipseParams effective = params;
    if (params.fast_mode) {
        effective.voxel_leaf = 0.03f;
        effective.circle_max_iter = 2000;
    }

    // Step 1: Segment circles in 2D (plane RANSAC + projection + circle RANSAC)
    SegmentationResult seg = segmentCircles2D(cloud, effective, plane_max_iter);
    result.plane_normal = seg.fitted_normal;
    result.plane_centroid = seg.fitted_centroid;

    if (!seg.success || seg.clusters_2d.empty()) {
        result.error = "Circle segmentation failed";
        result.error_code = seg.error_code;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // Step 2: Fit ellipse for each cluster
    int fit_ok_count = 0;
    for (size_t ci = 0; ci < seg.clusters_2d.size(); ++ci) {
        const auto& cluster_pts = seg.clusters_2d[ci];
        if (cluster_pts.size() < 10) continue;

        // Use circle RANSAC center/radius as LM init when available
        const Eigen::Vector2f* circle_center_ptr = nullptr;
        float circle_radius = 0.0f;
        if (ci < seg.circle_centers_2d.size() && ci < seg.circle_radii.size()) {
            circle_center_ptr = &seg.circle_centers_2d[ci];
            circle_radius = seg.circle_radii[ci];
        }

        Ellipse2D e2d = fitEllipse2DWithInit(cluster_pts, circle_center_ptr, circle_radius,
                                              effective.enable_lm_algebraic,
                                              effective.enable_lm_sampson,
                                              effective.near_circle_ratio);
        if (!e2d.valid) continue;

        result.ellipses.push_back(e2d);
        result.centers.push_back(lift2DCenterTo3D(e2d.center, seg.fitted_centroid,
                                                    seg.fitted_normal, seg.U, seg.V));
        ++fit_ok_count;
    }

    result.time_ms = timer.elapsedMs();

    if (result.centers.empty()) {
        result.error = "No valid ellipse fitted";
        result.error_code = EllipseError::FIT_FAIL;
        return result;
    }

    result.valid = true;
    return result;
}

// ===========================================================================
// MODE_C — PCL SACMODEL_ELLIPSE3D 3D RANSAC (precise)
//   Ported from benchmark runAlgoC (main.cpp lines 1297-1404)
// ===========================================================================

EllipseResult fitEllipsesPCL3D(CP cloud, const EllipseParams& params)
{
    EllipseResult result;
    result.valid = false;
    result.plane_normal = Eigen::Vector3f::Zero();
    result.plane_centroid = Eigen::Vector3f::Zero();
    result.error_code = EllipseError::OK;

    Timer timer;
    timer.start();

    if (!cloud || cloud->empty()) {
        result.error = "Input cloud is empty";
        result.error_code = EllipseError::EMPTY_CLOUD;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // Step 1: RANSAC plane segmentation (shared with MODE_D for consistent eval frame)
    const float plane_dist_threshold = 0.05f;
    const int plane_max_iter = 5000;
    pcl::SampleConsensusModelPlane<PointT>::Ptr plane_model(
        new pcl::SampleConsensusModelPlane<PointT>(cloud));
    pcl::RandomSampleConsensus<PointT> ransac(plane_model, plane_dist_threshold);
    ransac.setMaxIterations(plane_max_iter);
    if (!ransac.computeModel()) {
        result.error = "Plane RANSAC failed";
        result.error_code = EllipseError::PLANE_SEGMENT_FAIL;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    Eigen::VectorXf plane_coeffs;
    ransac.getModelCoefficients(plane_coeffs);
    result.plane_normal = Eigen::Vector3f(plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]).normalized();

    pcl::Indices inliers;
    ransac.getInliers(inliers);

    Eigen::Vector3f centroid(0, 0, 0);
    for (int idx : inliers) centroid += (*cloud)[idx].getVector3fMap();
    centroid /= float(inliers.size());
    result.plane_centroid = centroid;

    // Step 2: Extract non-plane points (ellipse features)
    CP non_plane(new CloudT);
    for (const auto& pt : cloud->points) {
        float dist = std::abs(pt.x * plane_coeffs[0] + pt.y * plane_coeffs[1]
                              + pt.z * plane_coeffs[2] + plane_coeffs[3]);
        if (dist > params.height_threshold) non_plane->push_back(pt);
    }
    non_plane->width = non_plane->size();
    non_plane->height = 1;

    if (non_plane->empty()) {
        result.error = "No non-plane points";
        result.error_code = EllipseError::NO_NON_PLANE_POINTS;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // Step 3: Voxel downsampling (pcl_leaf_size preserves ellipse shape detail)
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(non_plane);
    voxel.setLeafSize(params.pcl_leaf_size, params.pcl_leaf_size, params.pcl_leaf_size);
    CP cloud_ds(new CloudT);
    voxel.filter(*cloud_ds);

    if (cloud_ds->empty()) {
        result.error = "Empty cloud after voxel";
        result.error_code = EllipseError::NO_NON_PLANE_POINTS;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // Step 4: Sequentially extract 2 ellipses via PCL SACMODEL_ELLIPSE3D
    // Note: PCL 1.15.1 SACMODEL_ELLIPSE3D does not support setAxis/setMinMaxRadius.
    // The algorithm auto-estimates ellipse normal via 6-point sampling.
    CP remaining = cloud_ds;
    Eigen::Vector3f U, V;
    buildPlaneFrame(result.plane_normal, U, V);

    for (int ellipse_idx = 0; ellipse_idx < 2; ++ellipse_idx) {
        pcl::SampleConsensusModelEllipse3D<PointT>::Ptr model(
            new pcl::SampleConsensusModelEllipse3D<PointT>(remaining));
        pcl::RandomSampleConsensus<PointT> ransac_ell(model, params.circle_dist_threshold);
        ransac_ell.setMaxIterations(params.pcl_max_iter);
        ransac_ell.setProbability(0.99);

        if (!ransac_ell.computeModel()) {
            break;
        }
        Eigen::VectorXf coeffs;
        ransac_ell.getModelCoefficients(coeffs);
        if (coeffs.size() < 8) {
            result.error_code = EllipseError::FIT_FAIL;
            break;
        }

        Eigen::Vector3f center3d(coeffs[0], coeffs[1], coeffs[2]);
        result.centers.push_back(center3d);

        // Build 2D ellipse result: project 3D center to plane 2D coords
        // PCL ELLIPSE3D returns a/b directly; theta not provided (set 0, near-circle assumed)
        Ellipse2D e2d;
        e2d.a = coeffs[3];
        e2d.b = coeffs[4];
        e2d.theta = 0.0f;
        Eigen::Vector3f to_c = center3d - result.plane_centroid;
        Eigen::Vector3f n = result.plane_normal;
        to_c = to_c - to_c.dot(n) * n;
        e2d.center = Eigen::Vector2f(to_c.dot(U), to_c.dot(V));
        e2d.valid = true;
        result.ellipses.push_back(e2d);

        // Remove current ellipse inliers, continue to next
        pcl::Indices ell_inliers;
        ransac_ell.getInliers(ell_inliers);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remaining);
        pcl::IndicesPtr inliers_ptr(new pcl::Indices(ell_inliers));
        extract.setIndices(inliers_ptr);
        extract.setNegative(true);
        CP next(new CloudT);
        extract.filter(*next);
        remaining = next;
        if (remaining->empty()) break;
    }

    result.time_ms = timer.elapsedMs();

    if (result.centers.size() < 2) {
        result.error = "Failed to extract 2 ellipses";
        result.error_code = (result.error_code == EllipseError::OK)
                            ? EllipseError::FIT_FAIL : result.error_code;
        return result;
    }

    result.valid = true;
    return result;
}

// ===========================================================================
// P0-4: Top-level dispatcher — auto-select by params.mode, apply ROI, timing
// ===========================================================================

EllipseResult fitEllipsesOnPlane(CP cloud, const EllipseParams& params)
{
    EllipseResult result;
    result.valid = false;
    result.plane_normal = Eigen::Vector3f::Zero();
    result.plane_centroid = Eigen::Vector3f::Zero();
    result.error_code = EllipseError::OK;

    Timer timer;
    timer.start();

    if (!cloud || cloud->empty()) {
        result.error = "Input cloud is empty";
        result.error_code = EllipseError::EMPTY_CLOUD;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // ROI pre-processing (unified for both modes)
    CP effective_cloud = cloud;
    if (params.roi_enabled) {
        effective_cloud = cropROI(cloud, params);
        if (effective_cloud->empty()) {
            result.error = "ROI crop produced empty cloud (check roi_centers/roi_half_sizes)";
            result.error_code = EllipseError::NO_NON_PLANE_POINTS;
            result.time_ms = timer.elapsedMs();
            return result;
        }
    }

    // Dispatch by mode
    if (params.mode == EllipseFitMode::MODE_C) {
        result = fitEllipsesPCL3D(effective_cloud, params);
    } else {
        result = fitEllipsesOnPlane_D(effective_cloud, params);
    }

    // Override time_ms to include ROI crop cost (top-level total)
    result.time_ms = timer.elapsedMs();
    return result;
}

// ===========================================================================
// P0-2/P0-3: Parameter factory functions
// ===========================================================================

EllipseParams makeDefaultParams()
{
    // Backward-compatible defaults: no ROI, no fast mode, MODE_D
    return EllipseParams{};
}

EllipseParams makeProductionParams()
{
    EllipseParams p;
    p.mode = EllipseFitMode::MODE_D;       // Production default: fast
    p.fast_mode = true;                    // Enable <50ms preset
    p.roi_enabled = true;                  // Enable ROI crop
    // roi_centers / roi_half_sizes MUST be filled by caller from CAD model
    // (factory cannot know theoretical ellipse positions)
    return p;
}

// ===========================================================================
// P1-5/P1-6: Version & validation utilities
// ===========================================================================

const char* ellipseFittingVersionString()
{
    return RXS_ELLIPSE_FITTING_VERSION_STRING;
}

bool validateParams(const EllipseParams& params, std::string& err_msg)
{
    err_msg.clear();

    if (params.voxel_leaf <= 0) {
        err_msg = "voxel_leaf must be > 0";
        return false;
    }
    if (params.circle_r_min > params.circle_r_max) {
        err_msg = "circle_r_min must be <= circle_r_max";
        return false;
    }
    if (params.circle_max_iter <= 0) {
        err_msg = "circle_max_iter must be > 0";
        return false;
    }
    if (params.pcl_max_iter <= 0) {
        err_msg = "pcl_max_iter must be > 0";
        return false;
    }
    if (params.near_circle_ratio <= 0 || params.near_circle_ratio > 1.0f) {
        err_msg = "near_circle_ratio must be in (0, 1]";
        return false;
    }
    if (params.roi_enabled) {
        if (params.roi_half_factor <= 0) {
            err_msg = "roi_half_factor must be > 0 when roi_enabled";
            return false;
        }
        if (!params.roi_centers.empty() &&
            params.roi_half_sizes.size() != params.roi_centers.size()) {
            err_msg = "roi_half_sizes size must match roi_centers size";
            return false;
        }
        for (size_t i = 0; i < params.roi_half_sizes.size(); ++i) {
            if (params.roi_half_sizes[i] <= 0) {
                err_msg = "roi_half_sizes[" + std::to_string(i) + "] must be > 0";
                return false;
            }
        }
    }
    return true;
}

} // namespace rxs
