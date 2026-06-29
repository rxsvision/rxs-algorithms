#include "DerivedFittingSurface.h"
#include<pcl/surface/on_nurbs/nurbs_solve.h>


void DerivedFittingSurface::assemble(Parameter param)
{
    int nBnd = static_cast<int> (m_data->boundary.size());
    int nInt = static_cast<int> (m_data->interior.size());
    int nCurInt = param.regularisation_resU * param.regularisation_resV;
    int nCurBnd = 2 * param.regularisation_resU + 2 * param.regularisation_resV;
    int nCageReg = (m_nurbs.m_cv_count[0] - 2) * (m_nurbs.m_cv_count[1] - 2);
    int nCageRegBnd = 2 * (m_nurbs.m_cv_count[0] - 1) + 2 * (m_nurbs.m_cv_count[1] - 1);

    if (param.boundary_weight <= 0.0)
        nBnd = 0;
    if (param.interior_weight <= 0.0)
        nInt = 0;
    if (param.boundary_regularisation <= 0.0)
        nCurBnd = 0;
    if (param.interior_regularisation <= 0.0)
        nCurInt = 0;
    if (param.interior_smoothness <= 0.0)
        nCageReg = 0;
    if (param.boundary_smoothness <= 0.0)
        nCageRegBnd = 0;

    int ncp = m_nurbs.m_cv_count[0] * m_nurbs.m_cv_count[1];
    int nrows = nBnd + nInt + nCurInt + nCurBnd + nCageReg + nCageRegBnd;

    m_solver.assign(nrows, ncp, 3);

    unsigned row = 0;

    // boundary points should lie on edges of surface
    if (nBnd > 0)
        assembleBoundary(param.boundary_weight, row);

    // interior points should lie on surface
    if (nInt > 0)
        assembleInterior(param.interior_weight, row);

    // minimal curvature on surface
    if (nCurInt > 0)
    {
        if (m_nurbs.Order(0) < 3 || m_nurbs.Order(1) < 3)
            printf("[FittingSurface::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
        else
            addInteriorRegularisation(2, param.regularisation_resU, param.regularisation_resV,
                param.interior_regularisation / param.regularisation_resU, row);
    }

    // minimal curvature on boundary
    if (nCurBnd > 0)
    {
        if (m_nurbs.Order(0) < 3 || m_nurbs.Order(1) < 3)
            printf("[FittingSurface::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
        else
            addBoundaryRegularisation(2, param.regularisation_resU, param.regularisation_resV,
                param.boundary_regularisation / param.regularisation_resU, row);
    }

    // cage regularisation
    if (nCageReg > 0)
        addCageInteriorRegularisation(param.interior_smoothness, row);

    if (nCageRegBnd > 0)
    {
        addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::NORTH, row);
        addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::SOUTH, row);
        addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::WEST, row);
        addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::EAST, row);
        addCageCornerRegularisation(param.boundary_smoothness * 2.0, row);
    }
}