/**
 * @file dll_holeDeepth.cpp
 * @brief DLL export wrapper for holeDeepth — reference integration
 *
 * This file shows how to integrate the hole_depth module back into
 * czxToolkit.dll (rxs-toolkit). To activate:
 *
 * 1. Copy hole_depth.h and hole_depth.cpp into the rxs-toolkit source tree
 * 2. Add this file to the DLL project
 * 3. Add "holeDeepth" to Source.def EXPORTS section
 * 4. Rebuild czxToolkit.dll
 *
 * After this, the consumer-side GetProcAddress("holeDeepth") will succeed,
 * and the original protected_rxsToolKit wrapper will work as designed.
 *
 * @author RXS Vision
 * @license BSL 1.1
 */

#include "hole_depth.h"
#include <vector>
#include <cstring>

// Type definitions matching the consumer-side contract
// (from protected_rxsToolKit and rxsFixdal.h)

#pragma pack(push, 1)
struct rxsPointCould {
    float x;
    float y;
    float z;
};
#pragma pack(pop)

typedef rxsPointCould* rxsPointCouldp;

// Minimal result report interface (matches rxsResultReport usage pattern)
// The consumer code calls:
//   int ID = Result->AddRow();
//   int rowIndex = ID - 1;
//   Result->SetValue("ColumnName", "value", rowIndex);
// We use a C-style virtual interface for ABI compatibility.
class IResultReport {
public:
    virtual int  AddRow() = 0;
    virtual void SetValue(const char* colName, const char* value, int rowIndex) = 0;
};

// ---------------------------------------------------------------------------
// DLL export function
// ---------------------------------------------------------------------------

/**
 * @brief Measure hole depth — DLL export
 *
 * Signature matches the original holeDeepth contract:
 *   bool holeDeepth(rxsPointCouldp* cp, unsigned cpNums, rxsResultReport* Result, double step)
 *
 * @param cp      Array of point cloud data pointers (one per scan)
 * @param cpNums  Number of point clouds
 * @param Result  Result report interface for output
 * @param step    Sampling step for depth profile (mm)
 * @return true on success, false on failure
 */
extern "C" __declspec(dllexport)
bool holeDeepth(rxsPointCouldp* cp, unsigned cpNums, IResultReport* Result, double step) {
    if (!cp || cpNums < 1 || !Result) return false;

    // Convert input to PCL cloud
    // Note: original code processes the first cloud (cp[0])
    CP cloud(new CloudT);
    rxsPointCouldp points = cp[0];
    if (!points) return false;

    // Count points — we need to know the size
    // In the original framework, cpNums is the number of points in cp[0]
    // (not the number of separate clouds, despite the pointer-to-pointer signature)
    cloud->width = cpNums;
    cloud->height = 1;
    cloud->points.resize(cpNums);
    for (unsigned i = 0; i < cpNums; ++i) {
        cloud->points[i].x = points[i].x;
        cloud->points[i].y = points[i].y;
        cloud->points[i].z = points[i].z;
    }

    // Run hole depth measurement
    HoleDepthConfig config;
    config.step = step;
    auto results = measureHoleDepth(cloud, config);

    if (results.empty()) return false;

    // Format results into the report
    char buff[32];
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        int ID = Result->AddRow();
        int rowIndex = ID - 1;

        sprintf(buff, "%d", ID);
        Result->SetValue("ID", buff, rowIndex);

        sprintf(buff, "%.5f", r.centerX);
        Result->SetValue("孔中心X(mm)", buff, rowIndex);

        sprintf(buff, "%.5f", r.centerY);
        Result->SetValue("孔中心Y(mm)", buff, rowIndex);

        sprintf(buff, "%.5f", r.depth);
        Result->SetValue("孔深(mm)", buff, rowIndex);

        sprintf(buff, "%.5f", r.radius);
        Result->SetValue("孔半径(mm)", buff, rowIndex);

        sprintf(buff, "%.5f", r.surfaceZ);
        Result->SetValue("表面Z(mm)", buff, rowIndex);

        Result->SetValue("PASS", r.valid ? "OK" : "NG", rowIndex);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Source.def addition (append to existing EXPORTS section):
// ---------------------------------------------------------------------------
/*
   holeDeepth
*/
