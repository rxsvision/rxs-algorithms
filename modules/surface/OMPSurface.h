#pragma once
#include"czxTool.h"
#include <pcl/common/common.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include<pcl/surface/on_nurbs/fitting_surface_pdm.h>
#include"NurbsSurface.h"
#include"NurbsFit.h"

#define NEW_NURBS
//#define CZX_DEBUG


//#include "DerivedFittingSurface.h"

struct ROICloud
{
	CP cloud;
	float min_x, max_x, min_y, max_y;
	#ifdef NEW_NURBS
	NurbsFit* fit;
	#else
	pcl::on_nurbs::FittingSurface* fit;
	#endif
	float profile;
};

using VRC = vector<ROICloud>;

class OMPSurface
{
public:
	OMPSurface(int sqrted_zone_num);
	void setInputCloud(CP cloud);
	void process();
	
	~OMPSurface();
	bool devideCloud(CP cloud, int size_sub, VRC& output);

	template<typename T>
	CP getNurb(T nurbs);

	#ifdef NEW_NURBS
	bool fitting_2(ROICloud& cloud_in, int N);//重构之后的nurbs
	#else
	bool fitting(ROICloud& cloud_in, int N);//PCL的nurbs
	#endif

	template<typename T>
	void PointCloud2Vector3d(T cloud, pcl::on_nurbs::vector_vec3d& data);

	template<typename FIT= NurbsFit, typename CLOUD>
	double surfaceProfile(FIT& fit, CLOUD cloud);
	template<typename FIT= NurbsFit, typename CLOUD>
	bool Verify(FIT& fit, CLOUD cloud, double threshold_dis, double threshold_num);

	
	CP reconstruction();

	VRC sub_clouds;


private:
	template<typename FIT, typename CLOUD>
	CP reconstruction(FIT& fit, typename CLOUD cloud);

public:
	int zone_num;
	double max_profile;
	mutex mtx_subcloud;
	CP main_cloud;
	unordered_map<string, string> conf;
	//NurbsFit fit;

	#ifdef CZX_DEBUG
	mutex debug_mtx;
	#endif // CZX_DEBUG

};

