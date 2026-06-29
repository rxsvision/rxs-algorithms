#pragma once
#include"czxTool.h"
#include "Hole.hpp"

#include <pcl/common/common.h>
#include<pcl/filters/passthrough.h>
#include<pcl/io/pcd_io.h>
#include<pcl/common/pca.h>

#include <algorithm>
#include<vector>
#include<array>
#include<filesystem>
#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>

class BSplineLength
{
public:
	BSplineLength(string conf_path = "conf_curve.czx");
	bool verify(CP cloud, float min_length, float min_points, bool along_y);
	vector<CP> slice(const CP cloud, bool along_y);
	vector<CP> choice(vector<CP> clouds, int num = 1);
	Eigen::Matrix3f modelNormalize(CP cloud);
	vector<CP> getStripes(CP cloud, bool along_y);
	Vector2d differentialOne(MatrixXd pm, double tf);
	double curveLength(Eigen::MatrixXd controlMatrix);
	Vector2d getPos(MatrixXd pm, double tf);
	double curveLengthTest(Eigen::MatrixXd controlMatrix);
	vector<double> fitBSpline(CP cloud, bool along_y);
	vector<double> fitBSpline(CP cloud);
	void PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data);
	CP getNurbsCloud(const ON_NurbsCurve& nurbs);
	CP getNurbsCloudMy(const ON_NurbsCurve& nurbs);
	double lineProfile(const pcl::on_nurbs::FittingCurve2d& fit, CP cloud);
	double lineProfileMy(const pcl::on_nurbs::FittingCurve2d& fit, CP cloud);
	ON_3dPoint inverseMapping(pcl::KdTreeFLANN<PointT>& nurbs_tree, CP cloud, ON_3dPoint target);
	vector<vector<vector<double>>> process(CP cloud);
	double euclideanDistance(const double* array1, const double* array2, int dim);
	double curveLengthPcl(const ON_NurbsCurve& nurbs);
	double curveLengthGaussian(const ON_NurbsCurve& nurbs);
	MatrixXd getControl(const ON_NurbsCurve& nurbs);

public:
	ON_NurbsCurve nurbs_curve;

	unordered_map<string, string> conf;
	vector<CP> stripes_x;
	vector<CP> stripes_y;
	vector<CP> fitted_x;
	vector<CP> fitted_y;
	bool visual;
};
