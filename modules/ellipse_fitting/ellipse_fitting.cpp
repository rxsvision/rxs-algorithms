#include "ellipse_fitting.h"

// PCL
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>

// Eigen
#include <Eigen/Eigenvalues>

// STL
#include <random>
#include <numeric>
#include <algorithm>
#include <cmath>

namespace rxs {

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
// ---------------------------------------------------------------------------

/// Internal segmentation result
struct SegmentationResult {
    Eigen::Vector3f fitted_normal;
    Eigen::Vector3f fitted_centroid;
    Eigen::Vector3f U, V;                                  // Plane local frame
    std::vector<std::vector<Eigen::Vector2f>> clusters_2d; // 2D projected points per cluster
    std::vector<Eigen::Vector2f> circle_centers_2d;        // Circle RANSAC centers (LM init)
    std::vector<float> circle_radii;                       // Circle RANSAC radii (LM init)
    bool success;
};

/// RANSAC plane + non-plane extraction + voxel + 2D projection + circle RANSAC
static SegmentationResult segmentCircles2D(const CP& cloud,
                                           const EllipseParams& params)
{
    SegmentationResult result;
    result.success = false;
    if (cloud->empty()) return result;

    // Plane RANSAC parameters (fixed, matching benchmark defaults)
    const float plane_dist_threshold = 0.05f;
    const int plane_max_iter = 5000;

    // Step 1: RANSAC plane segmentation
    pcl::SampleConsensusModelPlane<PointT>::Ptr plane_model(
        new pcl::SampleConsensusModelPlane<PointT>(cloud));
    pcl::RandomSampleConsensus<PointT> ransac(plane_model, plane_dist_threshold);
    ransac.setMaxIterations(plane_max_iter);
    if (!ransac.computeModel()) return result;

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

    if (non_plane->empty()) return result;

    // Step 3: Plane local frame
    buildPlaneFrame(result.fitted_normal, result.U, result.V);

    // Step 4: Voxel downsampling
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(non_plane);
    voxel.setLeafSize(params.voxel_leaf, params.voxel_leaf, params.voxel_leaf);
    CP non_plane_ds(new CloudT);
    voxel.filter(*non_plane_ds);

    if (non_plane_ds->empty()) return result;

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

    if (result.clusters_2d.size() == 2) result.success = true;
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

// ---------------------------------------------------------------------------
// Public API: full 3D pipeline
// ---------------------------------------------------------------------------

EllipseResult fitEllipsesOnPlane(CP cloud, const EllipseParams& params)
{
    EllipseResult result;
    result.valid = false;
    result.plane_normal = Eigen::Vector3f::Zero();
    result.plane_centroid = Eigen::Vector3f::Zero();

    if (!cloud || cloud->empty()) {
        result.error = "Input cloud is empty";
        return result;
    }

    // Step 1: Segment circles in 2D (plane RANSAC + projection + circle RANSAC)
    SegmentationResult seg = segmentCircles2D(cloud, params);
    result.plane_normal = seg.fitted_normal;
    result.plane_centroid = seg.fitted_centroid;

    if (!seg.success || seg.clusters_2d.empty()) {
        result.error = "Circle segmentation failed";
        return result;
    }

    // Step 2: Fit ellipse for each cluster
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
                                              params.enable_lm_algebraic,
                                              params.enable_lm_sampson,
                                              params.near_circle_ratio);
        if (!e2d.valid) continue;

        result.ellipses.push_back(e2d);
        result.centers.push_back(lift2DCenterTo3D(e2d.center, seg.fitted_centroid,
                                                    seg.fitted_normal, seg.U, seg.V));
    }

    if (result.centers.empty()) {
        result.error = "No valid ellipse fitted";
        return result;
    }

    result.valid = true;
    return result;
}

} // namespace rxs
