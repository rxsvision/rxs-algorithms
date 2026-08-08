/**
 * @file profile.cpp
 * @brief Point cloud profile measurement module (skeleton implementation)
 *
 * @warning SKELETON ONLY. Original source code lost.
 *          All functions throw std::runtime_error.
 *          See docs/ for RD specs and legacy DLL SDK reference.
 */

#include "profile.h"
#include <stdexcept>

namespace rxs {

ProfileResult computeProfile(CP measured, CP nominal, const ProfileConfig& config) {
    (void)measured; (void)nominal; (void)config;
    throw std::runtime_error(
        "rxs::computeProfile() not implemented — original source lost. "
        "See docs/RD-03002-001 and docs/最小区域法轮廓度检测算子.docx for spec.");
}

std::vector<float> compareWithCAD(CP measured, CP nominal) {
    (void)measured; (void)nominal;
    throw std::runtime_error(
        "rxs::compareWithCAD() not implemented — original source lost.");
}

std::vector<float> getDeviation() {
    throw std::runtime_error(
        "rxs::getDeviation() not implemented — original source lost.");
}

} // namespace rxs
