#ifndef RXS_ELLIPSE_FITTING_H
#define RXS_ELLIPSE_FITTING_H

/**
 * @file ellipse_fitting.h
 * @brief Ellipse fitting module — Taubin AMS + two-step LM nonlinear refinement
 *
 * Pipeline:
 * 1. RANSAC plane segmentation → extract non-plane points → voxel downsampling
 * 2. Project to 2D plane → circle RANSAC to segment multiple ellipse clusters
 * 3. Taubin AMS algebraic fit → LM_algebraic (algebraic residual) → LM_sampson (Sampson distance)
 * 4. Near-circle constraint (force theta=0 when a/b > threshold)
 *
 * Algorithm source: benchmark v7 (D:\01_Environments_AImodels\benchmark\main.cpp)
 *   - fitEllipseTaubin: Taubin 1991 AMS, no generalized eigenvalue needed
 *   - refineEllipseLM_algebraic: analytic Jacobian, fastest convergence
 *   - refineEllipseLM_sampson: numerical Jacobian, approximate geometric distance
 *
 * @note Thread-safety: All functions are thread-safe (no global mutable state).
 *       Different threads can call fitEllipsesOnPlane concurrently with
 *       different params/clouds.
 *
 * @license BSL 1.1 (Change Date: 2030-01-01, Change License: GPLv2)
 * @company  Suzhou RXS Vision Technology Co., Ltd.
 */

// ---------------------------------------------------------------------------
// Version macros (P1-5)
// ---------------------------------------------------------------------------
#define RXS_ELLIPSE_FITTING_VERSION_MAJOR 1
#define RXS_ELLIPSE_FITTING_VERSION_MINOR 1
#define RXS_ELLIPSE_FITTING_VERSION_PATCH 0
#define RXS_ELLIPSE_FITTING_VERSION_STRING "1.1.0"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace rxs {

// Type aliases (consistent with coplanarity module)
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CP;

/// Ellipse fitting algorithm mode
enum class EllipseFitMode {
    MODE_D,  ///< Fast: RANSAC plane + Taubin AMS + two-step LM (~62ms, E~0.025mm)
    MODE_C,  ///< Precise: PCL SACMODEL_ELLIPSE3D 3D RANSAC (~205ms, E1~0.017mm)
};

/**
 * @brief Error codes for ellipse fitting (P0-1)
 *
 * Distinguishes failure causes so callers can decide retry vs alarm.
 */
enum class EllipseError {
    OK = 0,              ///< Success
    EMPTY_CLOUD,         ///< Input point cloud is empty
    INVALID_PARAMS,      ///< Invalid parameter values
    PLANE_SEGMENT_FAIL,  ///< Plane RANSAC failed to converge
    NO_NON_PLANE_POINTS, ///< No non-plane points (height_threshold too large)
    CLUSTER_FAIL,        ///< Circle RANSAC / DBSCAN clustering failed
    FIT_FAIL,            ///< Ellipse fitting failed (Taubin/LM did not converge)
    NUMERIC_INSTABLE,    ///< Numerical instability (singular matrix)
    UNKNOWN = 99
};

/**
 * @brief Ellipse fitting parameters
 *
 * Use makeDefaultParams() for backward-compatible defaults (no ROI, no fast mode).
 * Use makeProductionParams() for production deployment (ROI + fast mode enabled).
 */
struct EllipseParams {
    EllipseFitMode mode = EllipseFitMode::MODE_D;  ///< Algorithm mode (D=fast, C=precise)
    float height_threshold = 0.06f;      ///< Non-plane point height threshold
    float voxel_leaf = 0.02f;            ///< Voxel downsampling leaf size
    float circle_r_min = 1.5f;           ///< Circle RANSAC minimum radius
    float circle_r_max = 2.5f;           ///< Circle RANSAC maximum radius
    float circle_dist_threshold = 0.02f; ///< Circle RANSAC inlier distance threshold
    int circle_max_iter = 5000;          ///< Circle RANSAC maximum iterations
    bool enable_lm_algebraic = true;     ///< Enable algebraic residual LM
    bool enable_lm_sampson = true;       ///< Enable Sampson distance LM
    float near_circle_ratio = 0.95f;     ///< Near-circle constraint (force theta=0 when a/b > this)
    unsigned int seed = 42;              ///< RANSAC random seed (0 = random)
    // MODE_C specific
    float pcl_leaf_size = 0.05f;         ///< Voxel leaf for PCL ELLIPSE3D
    int pcl_max_iter = 10000;            ///< PCL RANSAC max iterations

    // --- P0-2: ROI configuration (built-in, no external preprocessing) ---
    bool roi_enabled = false;            ///< Enable ROI crop (default off for backward compat)
    float roi_half_factor = 2.0f;        ///< ROI half-side = max(theoretical_a, theoretical_b) * factor
    float roi_z_margin = 0.18f;          ///< Z margin to include non-plane noise thickness
    std::vector<Eigen::Vector3f> roi_centers; ///< ROI center points (theoretical ellipse centers)
    std::vector<float> roi_half_sizes;        ///< ROI half-sides (max(a,b) * roi_half_factor per ellipse)

    // --- P0-3: fast mode preset (target <50ms with ROI) ---
    bool fast_mode = false;              ///< Fast mode: plane_max_iter=2000, voxel=0.03, circle_iter=2000
};

/**
 * @brief Single 2D ellipse result
 */
struct Ellipse2D {
    Eigen::Vector2f center = Eigen::Vector2f::Zero(); ///< Ellipse center in 2D
    float a = 0;      ///< Semi-major axis (a >= b)
    float b = 0;      ///< Semi-minor axis
    float theta = 0;  ///< Rotation angle (radians)
    bool valid = false; ///< Whether fitting succeeded
};

/**
 * @brief Complete 3D ellipse fitting result
 */
struct EllipseResult {
    std::vector<Eigen::Vector3f> centers;     ///< 3D ellipse centers (lifted from 2D)
    std::vector<Ellipse2D> ellipses;          ///< 2D ellipse parameters (one per cluster)
    Eigen::Vector3f plane_normal;             ///< Fitted plane normal vector
    Eigen::Vector3f plane_centroid;           ///< Fitted plane centroid
    bool valid = false;                       ///< Whether computation succeeded
    std::string error;                        ///< Error message if !valid (backward compat)
    EllipseError error_code = EllipseError::OK; ///< Structured error code (P0-1)
    double time_ms = 0;                       ///< Total elapsed time (P0-4): ROI + segment + fit
};

// ---------------------------------------------------------------------------
// Parameter presets (P0-2/P0-3): two factory functions
// ---------------------------------------------------------------------------

/**
 * @brief Default parameters (backward-compatible, no ROI, no fast mode)
 *
 * Equivalent to EllipseParams{} default construction. Use this when you need
 * the legacy behavior (caller manages ROI externally, standard speed).
 *
 * @return EllipseParams with all defaults
 */
EllipseParams makeDefaultParams();

/**
 * @brief Production parameters (ROI enabled, fast mode on, target <50ms)
 *
 * Recommended for production deployment. Requires caller to fill in
 * roi_centers and roi_half_sizes based on CAD theoretical model.
 *
 * Mode is MODE_D (fast). To use MODE_C (precise), set .mode after construction:
 *   auto p = rxs::makeProductionParams();
 *   p.mode = rxs::EllipseFitMode::MODE_C;
 *
 * @return EllipseParams with roi_enabled=true, fast_mode=true, optimized iters
 */
EllipseParams makeProductionParams();

// ---------------------------------------------------------------------------
// Version & validation utilities (P1-5, P1-6)
// ---------------------------------------------------------------------------

/**
 * @brief Get library version string
 * @return Static C string like "1.1.0"
 */
const char* ellipseFittingVersionString();

/**
 * @brief Validate parameter values (P1-6)
 *
 * Checks: voxel_leaf > 0, circle_r_min <= circle_r_max, circle_max_iter > 0,
 * roi_half_factor > 0 (if roi_enabled), near_circle_ratio in (0, 1].
 *
 * @param params   Parameters to validate
 * @param err_msg  [out] Error description if invalid (empty if valid)
 * @return true if all parameters are valid
 */
bool validateParams(const EllipseParams& params, std::string& err_msg);

// ---------------------------------------------------------------------------
// Full 3D pipeline (auto-dispatch by params.mode)
// ---------------------------------------------------------------------------

/**
 * @brief Full pipeline (auto-select by params.mode)
 *
 * Dispatches to fitEllipsesOnPlane_D (MODE_D) or fitEllipsesPCL3D (MODE_C).
 * Applies ROI crop first if params.roi_enabled = true.
 * Fills result.time_ms with total elapsed time.
 *
 * @param cloud   Input point cloud (plane + ellipse features)
 * @param params  Fitting parameters
 * @return EllipseResult with 3D centers, 2D ellipse params, timing, error_code
 */
EllipseResult fitEllipsesOnPlane(CP cloud, const EllipseParams& params = EllipseParams());

/**
 * @brief MODE_D: RANSAC plane + Taubin + two-step LM (fast, ~62ms)
 *
 * If params.fast_mode = true, uses plane_max_iter=2000, voxel_leaf=0.03,
 * circle_max_iter=2000 (target <50ms with ROI).
 */
EllipseResult fitEllipsesOnPlane_D(CP cloud, const EllipseParams& params);

/**
 * @brief MODE_C: PCL SACMODEL_ELLIPSE3D 3D RANSAC (precise, ~205ms)
 *
 * Uses PCL built-in 3D ellipse RANSAC. Higher accuracy (E1~0.017mm) but slower.
 */
EllipseResult fitEllipsesPCL3D(CP cloud, const EllipseParams& params);

// ---------------------------------------------------------------------------
// 2D-only ellipse fitting
// ---------------------------------------------------------------------------

/**
 * @brief 2D-only ellipse fitting: Taubin AMS + two-step LM (on a known 2D point set)
 *
 * @param pts                   Input 2D points
 * @param enable_lm_algebraic   Enable algebraic residual LM refinement
 * @param enable_lm_sampson     Enable Sampson distance LM refinement
 * @param near_circle_ratio     Near-circle constraint threshold
 * @return Ellipse2D with center, a, b, theta
 */
Ellipse2D fitEllipse2D(const std::vector<Eigen::Vector2f>& pts,
                       bool enable_lm_algebraic = true,
                       bool enable_lm_sampson = true,
                       float near_circle_ratio = 0.95f);

/**
 * @brief Taubin AMS algebraic ellipse fitting (no LM)
 *
 * Solves standard eigenvalue problem (no generalized eigenvalue needed).
 * Reference: G. Taubin, IEEE TPAMI, 1991.
 *
 * @param pts     Input 2D points (>= 5 points)
 * @param coeffs  Output 6 coefficients [A, B, C, D, E, F] for Ax²+Bxy+Cy²+Dx+Ey+F=0
 * @return true on success
 */
bool fitEllipseTaubin(const std::vector<Eigen::Vector2f>& pts,
                      Eigen::Matrix<float, 6, 1>& coeffs);

/**
 * @brief LM refinement minimizing algebraic residual (analytic Jacobian)
 *
 * Residual: r_i = u_i²/a² + v_i²/b² - 1, where (u,v) is point in ellipse local frame.
 * Parameters: p = [cx, cy, a, b, theta]
 *
 * @param pts       Input 2D points
 * @param center    [in/out] Ellipse center
 * @param a         [in/out] Semi-major axis (initial = Taubin/circle RANSAC estimate)
 * @param b         [in/out] Semi-minor axis
 * @param theta     [in/out] Rotation angle
 * @param max_iter  Maximum LM iterations
 * @param cost_tol  Convergence threshold (cost drop below this = converged)
 * @return true on success
 */
bool refineEllipseLM_algebraic(const std::vector<Eigen::Vector2f>& pts,
                               Eigen::Vector2f& center, float& a, float& b, float& theta,
                               int max_iter = 100, float cost_tol = 1e-9f);

/**
 * @brief LM refinement minimizing Sampson distance (numerical Jacobian)
 *
 * Sampson residual: r_i = F_i / |∇F_i|, where F = u²/a² + v²/b² - 1.
 * Approximates geometric distance at lower cost than full point-to-curve distance.
 * Jacobian computed via forward differences (h = 1e-5).
 *
 * @param pts       Input 2D points
 * @param center    [in/out] Ellipse center
 * @param a         [in/out] Semi-major axis
 * @param b         [in/out] Semi-minor axis
 * @param theta     [in/out] Rotation angle
 * @param max_iter  Maximum LM iterations
 * @param cost_tol  Convergence threshold
 * @return true on success
 */
bool refineEllipseLM_sampson(const std::vector<Eigen::Vector2f>& pts,
                             Eigen::Vector2f& center, float& a, float& b, float& theta,
                             int max_iter = 100, float cost_tol = 1e-9f);

} // namespace rxs

#endif // RXS_ELLIPSE_FITTING_H
