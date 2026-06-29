#pragma once
//#include<pcl/surface/on_nurbs/fitting_surface_pdm.h>
#include"D:/PCL 1.10.1/include/pcl-1.10/pcl\\surface/on_nurbs/fitting_surface_pdm.h"


class DerivedFittingSurface :
    public pcl::on_nurbs::FittingSurface
{
public:

    // 显式调用基类构造函数
    DerivedFittingSurface(pcl::on_nurbs::NurbsDataSurface* data, const ON_NurbsSurface& ns) : pcl::on_nurbs::FittingSurface(data, ns) {
        // 其他的派生类构造逻辑
    }

    void assemble(Parameter param = Parameter()) override;
    

    pcl::on_nurbs::NurbsSolve m_solver;

};

