#include"czxTool.h"
#include"Hole.hpp"
#include"types_.h"
#include"BSplineLength.h"

#include"core/spline_curve_fitting.h"
#include"core/read_write_asc.h"

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
#include <unsupported/Eigen/CXX11/Tensor>
#include <pcl/surface/on_nurbs/nurbs_solve.h>
#include <pcl/filters/conditional_removal.h>

#define CZX_DEBUG
//#define SAVE
#define LOAD
//#define CZX_TEST

using namespace std;
using namespace Eigen;
using namespace arsenal;

unordered_map<string, string> conf = czxTool::readProfile("conf_curve.czx");

/// <summary>
/// 
/// </summary>
/// <param name="cloud"></param>
/// <param name="min_length">点云最短长度</param>
/// <param name="min_angle">点云最大夹角</param>
/// <param name="min_points">最少点云数量</param>
/// <param name="along_y">点云是否沿着y轴</param>
/// <returns></returns>
/// 
bool verify(CP cloud, float min_length, float min_points, bool along_y)
{
	//static bool once = false;
	if (cloud->size() < 3) return false;
	PointT minPoint;
	PointT maxPoint;
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);
	
	{
		float dif;
		if (along_y)
			dif = maxPoint.y - minPoint.y;
		else
			dif = maxPoint.x - minPoint.x;
		if (dif < min_length)
		{
			return false;
		}
	}
	
	if (cloud->size() < min_points) return false;
	return true;
}


//TODO：这个和bsp里面的不一样，它处理了渐变累加误差
bool filter(CP cloud, float min_angle, bool along_y)
{
	//CzxComparison dgs(cloud);
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 1e-7)));
	condrem.setCondition(condition);
	condrem.setInputCloud(cloud);
	//CP tmp(new CloudT);
	condrem.filter(*cloud);
	//Tool::showComparison(cloud, tmp, 1, 2);
	//*cloud = *tmp;


	#ifdef CZX_DEBUG
	//CzxComparison gsagsa(cloud);
	#endif

	bool (*compare)(PointT, PointT);
	if (along_y)
		compare = [](PointT first, PointT second) {return first.y < second.y; };
	else
		compare = [](PointT first, PointT second) {return first.x < second.x; };

	CP ret(new CloudT);
	sort(cloud->begin(), cloud->end(), compare);

	// 寻找正确的初始点,以角度为条件
	int i = 0;
	while (true)
	{
		Vector3f p1 = cloud->points[i].getArray3fMap();
		Vector3f p2 = cloud->points[i + 1].getArray3fMap();
		Vector3f p3 = cloud->points[i + 2].getArray3fMap();
		if (along_y)
		{
			p1[0] = 0;
			p2[0] = 0;
			p3[0] = 0;
		}
		else
		{
			p1[1] = 0;
			p2[1] = 0;
			p3[1] = 0;
		}
		Vector3f v1 = p2 - p1;
		Vector3f v2 = p3 - p2;

		float angle = pcl::getAngle3D(v1, v2, true);
		if (angle < min_angle)
		{
			ret->push_back(cloud->points[i]);
			ret->push_back(cloud->points[i + 1]);
			ret->push_back(cloud->points[i + 2]);
			i = i + 2;
			break;
		}
		else
		{
			i++;
		}
	}

	for (; i < cloud->size()-1; i++)
	{
		Vector3f p1 = cloud->points[i-1].getArray3fMap();
		Vector3f p2 = cloud->points[i].getArray3fMap();
		Vector3f p3 = cloud->points[i+1].getArray3fMap();

		if (along_y)
		{
			p2[0] = 0;
			p3[0] = 0;
		}
		else
		{
			p2[1] = 0;
			p3[1] = 0;
		}


		//float euclidean_distance = (p2 - p3).norm();
		float euclidean_distance;
		if (along_y)
			euclidean_distance = abs(p2[1] - p1[1]);
		else
			euclidean_distance = abs(p2[0] - p1[0]);
		//判断孔洞是否存在
		if (along_y)
		{
			if (euclidean_distance > stof(conf["max_pixel_dis_y"]))
				return false;
		}
		else
		{
			if (euclidean_distance > stof(conf["max_pixel_dis_x"]))
				return false;
		}

		Vector3f v1 = p2 - p1;
		Vector3f v2 = p3 - p2;

		float angle = pcl::getAngle3D(v1, v2, true);
		if (angle < 0 || angle>90)
		{
			cout << angle;
			//angle = 180 - angle;
			CP tmp(new CloudT);
			tmp->push_back(cloud->points[i - 1]);
			tmp->push_back(cloud->points[i]);
			tmp->push_back(cloud->points[i + 1]);
			Tool::showComparison(cloud, tmp);
		}
		if (angle < min_angle)
		{
			ret->push_back(cloud->points[i]);
		}
	}
	*cloud = *ret;
	return true;
}

#ifdef CZX_DEBUG
volatile bool debug = true;
#endif

vector<CP> slice(const CP cloud, bool along_y)
{
	CzxTimer dsgfdh(__func__);
	PointT minPoint;
	PointT maxPoint;
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);


	float step;
	if (along_y)
		step = stof(conf["step_x"]);
	else
		step = stof(conf["step_y"]);
	vector<CP> ret;

	int steps;
	if (along_y)
		steps = static_cast<int>((maxPoint.x - minPoint.x) / step);
	else
		steps = static_cast<int>((maxPoint.y - minPoint.y) / step);

	//#ifdef CZX_DEBUG
	//Tool::show(cloud);
	//#endif

	#pragma omp parallel for
	for (int star = 0; star<steps; star++)
	{
		CP stripe(new CloudT);
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		if (along_y)
		{
			pass.setFilterLimits(star * step + minPoint.x, star * step + minPoint.x + step);
			pass.setFilterFieldName("x");
		}
		else
		{
			pass.setFilterLimits(star * step + minPoint.y, star * step + minPoint.y + step);
			pass.setFilterFieldName("y");
		}
		pass.filter(*stripe);
		bool qualified;

		//#ifdef CZX_DEBUG
		//if(debug)
		//	Tool::showComparison(cloud, stripe, 1, 2);
		//#endif

		if(along_y)
			qualified = verify(stripe, 0.95*(maxPoint.y-minPoint.y), 100, along_y);
		else
			qualified = verify(stripe, 0.95*(maxPoint.x - minPoint.x), 100, along_y);
		if (!qualified) continue;

		filter(stripe, 30, along_y);

		#pragma omp critical
		ret.push_back(stripe);
	}
	return ret;
}

//要注意输入的点云数目不能为空
vector<CP> choice(vector<CP> clouds, int num=1)
{
	sort(clouds.begin(), clouds.end(), [](CP first, CP second) {return first->size() > second->size(); });
	if (clouds.size() < num) num = clouds.size();
	return vector<CP>(clouds.begin(), clouds.begin() + num);
}

Eigen::Matrix3f modelNormalize(CP cloud)
{
	CzxTimer fbhgdjusfgvuw(__func__);
	pcl::PCA<PointT> pca;
	pca.setInputCloud(cloud);
	Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
	preProcess pp("");
	CP keypoints = pp.process(cloud);
	cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);
	if (pp.state)
	{
		keypoints->getMatrixXfMap(3, 4, 0) = projection * keypoints->getMatrixXfMap(3, 4, 0);
		MatrixXf keypoints_2d_eigen = keypoints->getMatrixXfMap(3, 4, 0).block(0,0,2,3);
		VectorXf x_axis = keypoints_2d_eigen.col(1) - keypoints_2d_eigen.col(0);
		x_axis.normalize();
		VectorXf y_axis(2);
		y_axis << -x_axis[1] , x_axis[0];

		Matrix3f projection_xy = Matrix3f::Identity();
		projection_xy.block(0, 0, 1, 2) = x_axis.transpose();
		projection_xy.block(1, 0, 1, 2) = y_axis.transpose();
		cloud->getMatrixXfMap(3, 4, 0) = projection_xy * cloud->getMatrixXfMap(3, 4, 0);
		projection = projection_xy * projection;
	}
	#ifdef CZX_DEBUG
	pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
	#endif
	return projection;
}

void projection(CP cloud, bool along_y)
{
	CzxTimer dgasfgbh(__func__);
	if (along_y)
	{
		cloud->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).row(0).setZero();
	}else
	{
		cloud->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).row(0).setZero();
	}
}

vector<CP> process(CP cloud, bool along_y)
{
	vector<CP> stripes;
	PointT minPoint;
	PointT maxPoint;
	float step;
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);

	int region_num = stoi(conf["region_num"]);
	if(along_y)
		step = (maxPoint.x - minPoint.x) / region_num;
	else
		step = (maxPoint.y - minPoint.y) / region_num;


	//#ifdef CZX_DEBUG
	//vector<CP> regions;
	//#endif

	vector<vector<CP>> candidates;
	#pragma omp parallel for
	for (int i = 0; i < region_num; i++)
	{
		CP region_cloud(new CloudT);
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		if (along_y)
		{
			pass.setFilterLimits(i * step + minPoint.x, i * step + minPoint.x + step);
			pass.setFilterFieldName("x");
		}
		else
		{
			pass.setFilterLimits(i * step + minPoint.y, i * step + minPoint.y + step);
			pass.setFilterFieldName("y");
		}
		pass.filter(*region_cloud);
		{
			vector<CP> ret = slice(region_cloud, along_y);
			if (ret.size() <= 0)
				continue;
			vector<CP> best = choice(ret, region_num);
			#pragma omp critical
			{
				candidates.push_back(best);
				//stripes.push_back(best);
				//#ifdef CZX_DEBUG
				//regions.push_back(region_cloud);
				//#endif
			}
		}
	}	


	int cur_index = 0;
	while (stripes.size() < region_num && cur_index < region_num)
	{
		for (vector<CP> cand : candidates)
		{
			if (cand.size() <= cur_index) continue;
			stripes.push_back(cand[cur_index]);
		}
		cur_index++;
	}
	if (stripes.size() > region_num) stripes = vector<CP>(stripes.begin(), stripes.begin() + region_num);

	//#ifdef CZX_DEBUG
	//for (int i=0; i<stripes.size();i++)
	//{
	//	Tool::showComparison(regions[i], stripes[i]);
	//	Tool::show(stripes[i]);
	//}
	//#endif

	return stripes;
}

//将点云转换为矢量（2维）
void PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		pcl::PointXYZ& p = cloud->at(i);		
		if (!std::isnan(p.x) && !std::isnan(p.y))
			data.push_back(Eigen::Vector2d(p.x, p.y));
	}
}

VectorXd differentialOne(MatrixXd pm, double tf)
{
	MatrixXd cm(4, 4);
	cm << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 0, 3, 0,
		1, 4, 1, 0;
	MatrixXd  tm_diff(4, 1);
	tm_diff << 3 * tf * tf, 2 * tf, 1, 0;
	//cout << pm << endl;
	//cout << cm << endl;
	//cout << tm_diff << endl;
	VectorXd differential = pm * cm.transpose() * tm_diff / 6;
	//cout << differential;
	return differential;
}

double curveLength(Eigen::MatrixXd controlMatrix)
{
	int cols = controlMatrix.cols();
	if (cols <= 4)
		return numeric_limits<double>::quiet_NaN();

	double length = 0;
	double interval = stod(conf["interval"]); 
	//#pragma omp parallel for
	for (int i = 0; i <= cols-4; i++)
	{
		double total = 0;
		double tf;
		for (tf = 0; tf <= 1; tf += interval)
		{			
			Vector2d differential = differentialOne(controlMatrix.block(0, i, 2, 4), tf);
			total += differential.norm();
		}
		tf -= interval;
		total = 2 * total - differentialOne(controlMatrix.block(0, i, 2, 4), 0).norm() - differentialOne(controlMatrix.block(0, i, 2, 4), tf).norm();
		length += total;// / 2 * interval;
	}
	length = length / 2 * interval;
	return length;
}

VectorXd getPos(MatrixXd pm, double tf)
{
	MatrixXd cm(4, 4);
	cm << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 0, 3, 0,
		1, 4, 1, 0;
	MatrixXd  tm_diff(4, 1);
	tm_diff << tf * tf * tf, tf* tf, tf, 1;
	VectorXd pos = pm * cm.transpose() * tm_diff / 6;
	return pos;
}

Vector2d homogeneousMap(Vector3d in)
{
	if (in[2] > 1e-4 || in[2] < -1e-4)
		return Vector2d(in[0] / in[2], in[1] / in[2]);
	else
		throw logic_error("Zero weight");
}

double curveLengthRation(const ON_NurbsCurve& nurbs)
{
	if (nurbs.CVCount() < 4)
		throw logic_error("empty nurbs");

	Eigen::MatrixXd controlMatrix(3, nurbs.CVCount() + 4);

	{
		ON_4dPoint cp;
		nurbs.GetCV(0, cp);
		controlMatrix(0, 0) = cp.x;
		controlMatrix(1, 0) = cp.y;
		controlMatrix(2, 0) = cp.w;
		controlMatrix(0, 1) = cp.x;
		controlMatrix(1, 1) = cp.y;
		controlMatrix(2, 1) = cp.w;
		for (int i = 0; i < nurbs.CVCount(); i++)
		{
			//ON_3dPoint cp;
			nurbs.GetCV(i, cp);
			controlMatrix(0, i + 2) = cp.x;
			controlMatrix(1, i + 2) = cp.y;
			controlMatrix(2, i + 2) = cp.w;
			//#ifdef CZX_DEBUG
			//cout << cp.x << "  ";
			//#endif
		}
		//cout << endl;
		nurbs.GetCV(nurbs.CVCount() - 1, cp);
		controlMatrix(0, nurbs.CVCount() + 2) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 2) = cp.y;
		controlMatrix(2, nurbs.CVCount() + 2) = cp.w;
		controlMatrix(0, nurbs.CVCount() + 3) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 3) = cp.y;
		controlMatrix(2, nurbs.CVCount() + 3) = cp.w;
	}

	int cols = controlMatrix.cols();
	double length = 0;
	double interval = stod(conf["interval"]);
	//#pragma omp parallel for

	Vector2d last_point = homogeneousMap(getPos(controlMatrix.block(0, 0, 3, 4), 0));
	Vector2d cur_pos;
	for (int i = 0; i <= cols - 4; i++)
	{
		double total = 0;
		double tf;
		for (tf = 0; tf <= 1; tf += interval)
		{
			cur_pos = homogeneousMap(getPos(controlMatrix.block(0, i, 3, 4), tf));
			length += (cur_pos - last_point).norm();
			last_point = cur_pos;
		}
	}
	return length;
}

double curveLengthTest(Eigen::MatrixXd controlMatrix)
{
	double sum = 0;

	int cols = controlMatrix.cols();
	if (cols <= 4)
		return numeric_limits<double>::quiet_NaN();

	double length = 0;
	double interval = stod(conf["interval"]);
	//#pragma omp parallel for
	for (int i = 0; i <= cols - 4; i++)
	{
		double tf;
		Vector2d last_pos = getPos(controlMatrix.block(0, i, 2, 4), 0);
		for (tf = interval; tf <= 1; tf += interval)
		{
			Vector2d pos = getPos(controlMatrix.block(0, i, 2, 4), tf);
			sum += (pos - last_pos).norm();
			last_pos = pos;
		}
	}
	
	return sum;
}

double euclideanDistance(const double* array1, const double* array2, int dim) {
	double distance = 0.0;
	for (int i = 0; i < 2; ++i) {
		distance += std::pow(array1[i] - array2[i], 2);
	}

	return std::sqrt(distance);
}

double curveLengthPcl(const ON_NurbsCurve &nurbs)
{
	double interval = stod(conf["interval"])/100;
	double last_point[2];
	double point[2];
	double sum = 0;
	nurbs.Evaluate(0, 0, 2, last_point);
	double last_knot = nurbs.Knot(nurbs.KnotCount()-1);
	for (double i = interval; i <= last_knot; i += interval)
	{
		nurbs.Evaluate(i, 0, 2, point);

		sum += euclideanDistance(point, last_point, 2);

		for (int i = 0; i < 2; i++)
		{
			last_point[i] = point[i];
		}
	}
	return sum;
}

/*注意释放内存，高阶仍未实现，这里的n和m不是教程里的n和m,而是它们+1*/
double* getNip(const ON_NurbsCurve& nurbs, double u, int i)
{
	int nKnots = nurbs.KnotCount();
	int nCPs = nurbs.CVCount();
	int degree = nurbs.Order() - 1;
	if (nKnots != nCPs + 2 * degree - 4)
	{
		//throw logic_error("m!=n+p+1");
		throw logic_error("m!=(n-2) + 2 * p - 2");
	}

	//if (nurbs.Knot(0) > u || nurbs.Knot(nKnots - 1) < u)
	//{
	//	throw logic_error("u not in [u0, um]");
	//}
	
	auto U1 = [nurbs, u](int i_, int degree_) {
		if (i_ < 0) return 0.0;
		double uip = nurbs.Knot(i_ + degree_);
		double ui;
		ui = nurbs.Knot(i_);
		return (u - ui) / (uip - ui);
	};

	auto U2 = [nurbs, u](int i_, int degree_) {
		if (i_ < 0) return 0.0;
		double uip1 = nurbs.Knot(i_ + degree_ + 1);
		double ui1 = nurbs.Knot(i_ + 1);
		return (uip1 - u) / (uip1 - ui1);
	};


	//TODO:实现自适应计算高阶

	//Ni-1,1,Ni,1
	volatile double N1[2];
	N1[0] = U2(i-1, 1);
	N1[1] = U1(i, 1);

	//Ni-2,2 ...
	double * N2 = new double[3];
	N2[0] = U2(i - 2, 2) * N1[0];
	N2[1] = U1(i - 1, 2) * N1[0] + U2(i - 1, 2) * N1[1];
	N2[2] = U1(i, 2) * N1[1];
	if (degree == 2) return N2;


	//Ni-3,3 ...
	double * N3 = new double[4];
	N3[0] = U2(i - 3, 3) * N2[0];
	N3[1] = U1(i - 2, 3) * N2[0] + U2(i - 2, 3) * N2[1];
	N3[2] = U1(i - 1, 3) * N2[1] + U2(i - 1, 3) * N2[2];
	N3[3] = U1(i, 3) * N2[2];
	if (degree == 3 && i!=2) return N3;
	if (degree == 3 && i == 2)
	{
		N3[0] = 1 - N3[1] - N3[2] - N3[3];
		return N3;
	}

	throw logic_error("unimplement");
	return nullptr;
}

ON_4dPoint Evaluate(const ON_NurbsCurve & nurbs, double u)
{
	int nKnots = nurbs.KnotCount();
	int nCPs = nurbs.CVCount();
	int degree = nurbs.Order() - 1;
	if (nKnots != nCPs + 2 * degree - 4)
	{
		//throw logic_error("m!=n+p+1");
		throw logic_error("m!=(n-2) + 2 * p - 2");
	}

	//假设均匀步长为1/(n-p+2),前后有p个重复	
	int i = floor(u / (1.0 / (nCPs - degree + 2))) + degree - 1;

	while (nurbs.Knot(i) > u || nurbs.Knot(i + 1) <= u) {
		if (nurbs.Knot(i) > u)
			i--;
		else
			i++;
	}

	double nip_c[4];
	double * nip = getNip(nurbs, u, i);
	for (int i = 0; i < 4; i++)
	{
		nip_c[i] = *(nip + i);
	}
	
	
	ON_4dPoint p(0, 0, 0, 0);
	for (int bias = 0; bias < degree + 1; bias++)
	{
		double &Nip = nip[bias];
		int index = i + bias - degree + 1;
		if (index < 0) continue;
		ON_4dPoint cur_p;
		nurbs.GetCV(index, cur_p);

		p.x += cur_p.x * Nip;
		p.y += cur_p.y * Nip;
		p.z += cur_p.z * Nip;
		p.w += cur_p.w * Nip;
	}
	
	p.x = p.x / p.w;
	p.y = p.y / p.w;
	p.z = p.z / p.w;
	p.w = p.w / p.w;
	
	delete[] nip;

	return p;
}

//pcl生成的nurb
CP getNurbsCloud(const ON_NurbsCurve & nurbs)
{
	CP ret(new CloudT);
	double interval = stod(conf["interval"]) / 100;
	double point[2];
	volatile double last_knot = nurbs.Knot(nurbs.KnotCount() - 1);
	for (double i = 0; i <= last_knot; i += interval)
	{
		nurbs.Evaluate(i, 0, 2, point);

		ret->push_back(PointT(point[0], point[1], 0));

	}
	return ret;
}

CP nurbsLength_CP_ration_NonUniform(const ON_NurbsCurve & nurbs)
{
	CP ret(new CloudT);
	double interval = stod(conf["interval"]) / 100;
	ON_4dPoint last_point;
	ON_4dPoint point;
	last_point = Evaluate(nurbs, 0);	
	volatile double last_knot = nurbs.Knot(nurbs.KnotCount() - 1);
	for (double i = interval; i <= last_knot; i += interval)
	//for (double i = nurbs.Knot(4); i <= last_knot; i += interval)
	{
		point = Evaluate(nurbs, i);
		//double point_[2];
		//nurbs.Evaluate(i, 0, 2, point_);
		//if (point_[0] - point.x > 0.1)
		//	cout << point.x;
		ret->push_back(PointT(point.x, point.y, point.z));
		//if (abs(point_[0] - point.x) > 0.001)
		//	cout << i << "  ";
		last_point = point;		
	}
	return ret;
}


#ifdef CZX_DEBUG
CP nurbsCurve(const ON_NurbsCurve& nurbs)
{
	if (nurbs.CVCount() < 4)
		throw logic_error("empty nurbs");

	Eigen::MatrixXd controlMatrix(3, nurbs.CVCount() + 4);

	{
		ON_4dPoint cp;
		nurbs.GetCV(0, cp);
		controlMatrix(0, 0) = cp.x;
		controlMatrix(1, 0) = cp.y;
		controlMatrix(2, 0) = cp.w;
		controlMatrix(0, 1) = cp.x;
		controlMatrix(1, 1) = cp.y;
		controlMatrix(2, 1) = cp.w;
		for (int i = 0; i < nurbs.CVCount(); i++)
		{
			//ON_3dPoint cp;
			nurbs.GetCV(i, cp);
			controlMatrix(0, i + 2) = cp.x;
			controlMatrix(1, i + 2) = cp.y;
			controlMatrix(2, i + 2) = cp.w;
			//#ifdef CZX_DEBUG
			//cout << cp.x << "  ";
			//#endif
		}
		//cout << endl;
		nurbs.GetCV(nurbs.CVCount() - 1, cp);
		controlMatrix(0, nurbs.CVCount() + 2) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 2) = cp.y;
		controlMatrix(2, nurbs.CVCount() + 2) = cp.w;
		controlMatrix(0, nurbs.CVCount() + 3) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 3) = cp.y;
		controlMatrix(2, nurbs.CVCount() + 3) = cp.w;
	}

	CP ret(new CloudT);

	int cols = controlMatrix.cols();
	double length = 0;
	double interval = stod(conf["interval"]);
	//#pragma omp parallel for
	for (int i = 0; i <= cols - 4; i++)
	{
		double tf;
		for (tf = 0; tf <= 1; tf += interval)
		{
			//MatrixXd tmp = controlMatrix.block(0, i, 2, 4);
			//cout << tmp;
			Vector3d pos = getPos(controlMatrix.block(0, i, 3, 4), tf);
			if (pos[2] > 1e-4 || pos[2] < -1e-4)
			{
				PointT p(pos[0] / pos[2], pos[1] / pos[2], 0);
				ret->push_back(p);
			}
		}
	}

	return ret;
}
#endif

#ifdef CZX_DEBUG
CP nurbsCurveTwo(const ON_NurbsCurve& nurbs, double bias=0.02)
{
	if (nurbs.CVCount() < 4)
		throw logic_error("empty nurbs");

	Eigen::MatrixXd controlMatrix(3, nurbs.CVCount() + 4);

	{
		ON_4dPoint cp;
		nurbs.GetCV(0, cp);
		controlMatrix(0, 0) = cp.x;
		controlMatrix(1, 0) = cp.y;
		controlMatrix(2, 0) = cp.w;
		controlMatrix(0, 1) = cp.x;
		controlMatrix(1, 1) = cp.y;
		controlMatrix(2, 1) = cp.w;
		for (int i = 0; i < nurbs.CVCount(); i++)
		{
			//ON_3dPoint cp;
			nurbs.GetCV(i, cp);
			controlMatrix(0, i + 2) = cp.x;
			controlMatrix(1, i + 2) = cp.y;
			controlMatrix(2, i + 2) = cp.w;
			//#ifdef CZX_DEBUG
			//cout << cp.x << "  ";
			//#endif
		}
		//cout << endl;
		nurbs.GetCV(nurbs.CVCount() - 1, cp);
		controlMatrix(0, nurbs.CVCount() + 2) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 2) = cp.y;
		controlMatrix(2, nurbs.CVCount() + 2) = cp.w;
		controlMatrix(0, nurbs.CVCount() + 3) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 3) = cp.y;
		controlMatrix(2, nurbs.CVCount() + 3) = cp.w;
	}

	CP ret(new CloudT);

	int cols = controlMatrix.cols();
	double length = 0;
	double interval = stod(conf["interval"]);
	//#pragma omp parallel for
	for (int i = 0; i <= cols - 4; i++)
	{
		double tf;
		for (tf = 0; tf <= 1; tf += interval)
		{
			//MatrixXd tmp = controlMatrix.block(0, i, 2, 4);
			//cout << tmp;
			Vector3d pos = getPos(controlMatrix.block(0, i, 3, 4), tf);
			if (pos[2] > 1e-4 || pos[2] < -1e-4)
			{
				PointT p(pos[0] / pos[2], pos[1] / pos[2], 0);
				ret->push_back(p);
				ret->push_back(PointT(p.x, p.y + bias, 0));
				ret->push_back(PointT(p.x, p.y - bias, 0));
			}
		}
	}

	return ret;
}
#endif

MatrixXd nurbsCPs2matrix(const ON_NurbsCurve& curve)
{
	Eigen::MatrixXd controlMatrix(2, curve.CVCount() + 4);
	ON_3dPoint cp;
	curve.GetCV(0, cp);
	controlMatrix(0, 0) = cp.x;
	controlMatrix(1, 0) = cp.y;
	controlMatrix(0, 1) = cp.x;
	controlMatrix(1, 1) = cp.y;
	for (int i = 0; i < curve.CVCount(); i++)
	{
		//ON_3dPoint cp;
		curve.GetCV(i, cp);
		controlMatrix(0, i + 2) = cp.x;
		controlMatrix(1, i + 2) = cp.y;
		//#ifdef CZX_DEBUG
		//cout << cp.x << "  ";
		//#endif
	}
	//cout << endl;
	curve.GetCV(curve.CVCount() - 1, cp);
	controlMatrix(0, curve.CVCount() + 2) = cp.x;
	controlMatrix(1, curve.CVCount() + 2) = cp.y;
	controlMatrix(0, curve.CVCount() + 3) = cp.x;
	controlMatrix(1, curve.CVCount() + 3) = cp.y;
	return controlMatrix;
}

Eigen::VectorXcd roots(VectorXd p)
{
	p = p / p(0);		//转换为最高次系数为1的标准形式

	MatrixXd matrixXd(p.size() - 1, p.size() - 1);
	for (int i = 0; i < p.size() - 1; ++i) {
		matrixXd(i, 0) = -p(i + 1) / p(0);
		for (int j = 1; j < p.size() - 1; ++j) {
			if (i + 1 == j)
				matrixXd(i, j) = 1;
			else
				matrixXd(i, j) = 0;
		}
	}
	Eigen::VectorXcd evalue = matrixXd.eigenvalues();
	//    cout << evalue << endl;
	return evalue;	//返回求解的特征值，这里是复数形式
}

//这里有个基本假设是控制点的x是单调递增的
//TODO这里寻找分区需要重写
ON_3dPoint inverseMapping(const ON_NurbsCurve& nurbs, ON_3dPoint target)
{
	int nCPs = nurbs.CVCount();
	MatrixXd control = nurbsCPs2matrix(nurbs);
	int hint = -1;
	for (int i = 0; i < nCPs+1; i++)
	{
		double m_x;
		//m_x = (control(i, 0) + control(i + 2, 0)) / 2;
		//double last = (m_x - control(i + 1, 0)) / 3 + control(i + 1, 0);
		m_x = (control(i+1, 0) + control(i + 3, 0)) / 2;
		double next = (m_x - control(i + 2, 0)) / 3 + control(i + 2, 0);
		if (target.x < next)
		{
			hint = i;
			break;
		}
	}
	//if(hint==-1 && abs(nurbs.CV(0)[0] - target.x) <1e-3)
	if (hint == -1) 
	{
		if (abs(nurbs.CV(0)[0] - target.x) < 1e-3) hint = 0;
		else if(abs(nurbs.CV(nCPs-1)[0] - target.x) < 1e-3) hint = nCPs-1;
		else throw logic_error("曲线上找不到该点");
	}		


	for(int _=0;_<2;_++)
	{
		MatrixXd pm = control.block(0, hint, 2, 4);

		MatrixXd cm(4, 4);
		cm << -1, 3, -3, 1,
			3, -6, 0, 4,
			-3, 3, 3, 1,
			1, 0, 0, 0;

		MatrixXd A = pm * cm / 6;
		Eigen::VectorXd B(2);
		B << target.x, target.y;

		// 定义列向量x来存储解
		Eigen::VectorXd x;

		Vector4d coff = A.row(0);
		coff(3) -= B(0);
		Eigen::Vector3cd root = roots(coff);

		int count = 0;
		double root_real = -1;
		bool found = false;
		for (int i = 0; i < 3; i++)
		{
			if (root[i].imag() == 0)
			{
				if (root_real > -1 && root_real < 2)
				{
					root_real = root[i].real();
					found = true;
					if(root_real>0 && root_real <1)
						break;
				}
			}
			//test
			//if (count >= 2)
			//{
			//	cout<< root;
			//}
		}
		if (!found)
		{
			float min_bias=999;
			double tmp_real;
			for (int i = 0; i < 3; i++)
			{
				if (root[i].imag() == 0)
				{
					found = true;
					tmp_real = root[i].real();
					if (abs(tmp_real - 0.5) < min_bias)
					{
						root_real = tmp_real;
						min_bias = abs(tmp_real - 0.5);
					}
				}
			}
			if(!found)
				throw logic_error("寻找最近点,找不到根");
		}
 		if (root_real > 1) hint++;
		else if (root_real < 0) hint--;
		else
		{
			Vector4d tm(root_real * root_real * root_real, root_real * root_real, root_real, 1);
			Vector2d nearestPoint = A * tm;
			return ON_3dPoint(nearestPoint[0], nearestPoint[1], 0);
		}
		if (hint<0 || hint>nCPs - 1)
		{
			Vector4d tm(root_real * root_real * root_real, root_real * root_real, root_real, 1);
			Vector2d nearestPoint = A * tm;
			return ON_3dPoint(nearestPoint[0], nearestPoint[1], 0);

		}
	}
	throw logic_error("找不到最近点");
	//// 使用Eigen的线性方程求解器来求解Ax = B
	//x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
	//cout << x<<endl;
	//cout << A << endl;
	//cout << B << endl;
}

ON_3dPoint inverseMapping(pcl::KdTreeFLANN<PointT>& nurbs_tree, CP cloud, ON_3dPoint target)
{
	std::vector<int> pointIdxNKNSearch; // 用于存储最近邻点的索引
	std::vector<float> pointNKNSquaredDistance; // 用于存储最近邻点的距离的平方
	if (nurbs_tree.nearestKSearch(PointT(target.x, target.y, target.z), 2, pointIdxNKNSearch, pointNKNSquaredDistance) < 2)
		throw logic_error("找不到近邻");
	return ON_3dPoint(cloud->points[pointIdxNKNSearch[1]].x, cloud->points[pointIdxNKNSearch[1]].y, cloud->points[pointIdxNKNSearch[1]].z);
}

#ifdef CZX_DEBUG
//手撸的
CP getCurve(Eigen::MatrixXd controlMatrix)
{
	CP ret(new CloudT);

	int cols = controlMatrix.cols();

	
	double length = 0;
	double interval = stod(conf["interval"]);
	//#pragma omp parallel for
	for (int i = 0; i <= cols - 4; i++)
	{
		double tf;
		for (tf = 0; tf <= 1; tf += interval)
		{
			//MatrixXd tmp = controlMatrix.block(0, i, 2, 4);
			//cout << tmp;
			Vector2d pos = getPos(controlMatrix.block(0, i, 2, 4), tf);
			PointT p(pos[0], pos[1], 0);
			ret->push_back(p);
		}
	}
	
	return ret;
}
#endif

//#ifdef CZX_DEBUG
//pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
//#endif
#ifdef CZX_DEBUG
void VisualizeCurve(const ON_NurbsCurve& curve,  CP origin, bool show_cps=true)
{
	pcl::visualization::PCLVisualizer viewer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 20);

	//for (int i = 0; i < curve.KnotCount(); i++)
	//{
	//	cout << curve.Knot(i) << " ";
	//}
	//cout << curve.KnotCount() << endl;
	//cout << endl;
	//cout << curve.ControlPolygonLength() << endl;;
	//cout << curve.IsClosed() << endl;;
	//double sum = 0;
	////添加线对象
	//for (std::size_t i = 0; i < cloud->size() - 1; i++)
	//{
	//	pcl::PointXYZRGB& p1 = cloud->at(i);
	//	pcl::PointXYZRGB& p2 = cloud->at(i + 1);
	//	std::ostringstream os;
	//	os << "line_" << r << "_" << g << "_" << b << "_" << i;
	//	viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
	//	sum += pcl::euclideanDistance(p1, p2);
	//}

	//cout << sum << endl;
	//viewer.addPointCloud(cloud, "sample");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample");

	//viewer.addPointCloud(nurbsLength_CP_ration_NonUniform(curve), "sample");
	//viewer.addPointCloud(getNurbsCloud(curve), "sample");
	viewer.addPointCloud(nurbsCurve(curve), "sample");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "sample");

	viewer.addPointCloud(origin, "origin");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "origin");

	//是否显示控制点
	if (show_cps)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < curve.CVCount(); i++)
		{
			ON_3dPoint cp;
			curve.GetCV(i, cp);

			{
				pcl::PointXYZ p;
				p.x = float(cp.x);
				p.y = float(cp.y);
				p.z = float(cp.z);
				cps->push_back(p);
			}
			//cout << p.x << "--";
		}
		cout << endl;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 0, 0, 255);
		viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_cps");

	}

	//////添加控制点线对象
	//for (std::size_t i = 0; i < curve.CVCount()-1; i++)
	//{
	//	ON_3dPoint cp;
	//	curve.GetCV(i, cp);
	//	pcl::PointXYZRGB p1(cp.x, cp.y, cp.z,1,1,1);
	//	curve.GetCV(i+1, cp);
	//	pcl::PointXYZRGB p2(cp.x, cp.y, cp.z, 1, 1, 1);
	//	std::ostringstream os;
	//	os << "cp_line_" << r << "_" << g << "_" << b << "_" << i;
	//	viewer.addLine(p1, p2, os.str());
	//}

	viewer.spin();
}
#endif

double lineProfile(const pcl::on_nurbs::FittingCurve2d& fit, CP cloud)
{

	CP curve_cloud = getNurbsCloud(fit.m_nurbs);
	pcl::KdTreeFLANN<PointT> curve_kdtree;
	curve_kdtree.setInputCloud(curve_cloud);
	double max_error = 0;
	int arg_max_error;
	for (int j = 0; j < fit.m_data->interior.size(); j++)
	{
		ON_3dPoint target(fit.m_data->interior[j][0], fit.m_data->interior[j][1], 0);
		double error = inverseMapping(curve_kdtree, curve_cloud, target).DistanceTo(target);
		if (error < max_error)
			continue;
		else
		{
			max_error = error;
			arg_max_error = j;
		}
	}

	//#ifdef CZX_DEBUG
	//CP error_region(new CloudT);
	//for (int i = arg_max_error - 5; i < arg_max_error + 5; i++)
	//{
	//	error_region->push_back(cloud->points[i]);
	//}
	//Tool::showComparison(cloud, error_region);

	//filter(error_region, 1, false);
	//#endif


	#ifdef CZX_DEBUG
	cout << "轮廓度" << 2 * max_error << endl;
	if (max_error > 0.02)
	{
		ON_3dPoint target(fit.m_data->interior[arg_max_error][0], fit.m_data->interior[arg_max_error][1], 0);
		ON_3dPoint pt = inverseMapping(curve_kdtree, curve_cloud, target);
		CP maxErrorPoint(new CloudT());
		maxErrorPoint.reset(new CloudT);
		maxErrorPoint->push_back(PointT(fit.m_data->interior[arg_max_error][0], fit.m_data->interior[arg_max_error][1], 0));
		maxErrorPoint->push_back(PointT(pt.x, pt.y, 0));
		Tool::showComparison(cloud, nurbsCurve(fit.m_nurbs), maxErrorPoint, 3, 1, 5);
		//Tool::showComparison(cloud, maxErrorPoint, 2, 5);
	}
	#endif
	return 2 * max_error;
}

// 会把点云变成xy0格式的2D点云
double fitBSpline(CP cloud, bool along_y)
{
	CzxTimer sdhfadh(__func__);
	unsigned order(4);		//阶数
	unsigned n_control_points(cloud->size()/stoi(conf["controlDense"]) + 4);		//控制点个数

	pcl::on_nurbs::NurbsDataCurve2d data;
	if (along_y)
	{
		cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1)); //y轴向x轴迁移
	}
	cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));
	#ifdef CZX_DEBUG
	float axis_z = cloud->getMatrixXfMap(3, 4, 0).row(2).mean();
	#endif
	cloud->getMatrixXfMap(3, 4, 0).row(2).setZero();

	sort(cloud->begin(), cloud->end(), [](PointT first, PointT second) {return first.x < second.x; });

	PointCloud2Vector2d(cloud, data.interior);

	pcl::on_nurbs::FittingCurve2d::Parameter curve_params;
	curve_params.smoothness = 0.01;		//光滑度
	curve_params.rScale = 0.02;		//尺度


	ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2d::initNurbsPCA(order, &data, n_control_points);

	curve.MakeRational();
	pcl::on_nurbs::FittingCurve2d fit(&data, curve);

	for (int i = 0; i < 1; i++)
	{
		fit.assemble(curve_params);		//装配曲线

		//设置固定点
		Eigen::Vector2d star(cloud->points[0].x, cloud->points[0].y);
		Eigen::Vector2d end(cloud->points[cloud->size() - 1].x, cloud->points[cloud->size() - 1].y);
		if (star[0] > end[0])
		{
			swap(star, end);
		}
		fit.addControlPointConstraint(0, star, 100);
		fit.addControlPointConstraint(curve.CVCount() - 1, end, 100); // 小一点的权重不一定会让最终控制点是首尾点	
		double diff = fit.solve();
		curve = fit.m_nurbs;
	}

	lineProfile(fit, cloud);

	Eigen::MatrixXd controlMatrix (2, curve.CVCount()+4);
	curve = fit.m_nurbs;

	{
		ON_3dPoint cp;
		curve.GetCV(0, cp);
		controlMatrix(0, 0) = cp.x;
		controlMatrix(1, 0) = cp.y;
		controlMatrix(0, 1) = cp.x;
		controlMatrix(1, 1) = cp.y;
		for (int i = 0; i < curve.CVCount(); i++)
		{
			//ON_3dPoint cp;
			curve.GetCV(i, cp);
			controlMatrix(0, i + 2) = cp.x;
			controlMatrix(1, i + 2) = cp.y;
			//#ifdef CZX_DEBUG
			//cout << cp.x << "  ";
			//#endif
		}
		//cout << endl;
		curve.GetCV(curve.CVCount() - 1, cp);
		controlMatrix(0, curve.CVCount() + 2) = cp.x;
		controlMatrix(1, curve.CVCount() + 2) = cp.y;
		controlMatrix(0, curve.CVCount() + 3) = cp.x;
		controlMatrix(1, curve.CVCount() + 3) = cp.y;
	}

	cout << "curveLengthNurb_pcl:" << curveLengthPcl(fit.m_nurbs) << endl;
	cout << "curveLengthNurb_my:" << curveLengthRation(fit.m_nurbs) << endl;

	#ifdef CZX_DEBUG
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudColor(cloud, 0, 255, 255);
	//viewer.addPointCloud<pcl::PointXYZ>(cloud, cloudColor, "cloud");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	//VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, true);
	//viewer.spin();
	//Tool::showComparison(cloud, getCurve(controlMatrix),2, 1);
	//Tool::showComparison(cloud, nurbsCurve(fit.m_nurbs), 2, 1);
	//Tool::showComparison(cloud, getNurbsCloud(fit.m_nurbs), 2, 1);


	//fit.m_nurbs.SetWeight(fit.m_nurbs.CVCount()-1, 100);
	//fit.m_nurbs.SetWeight(fit.m_nurbs.CVCount() - 2, 100);
	//double w = fit.m_nurbs.Weight(fit.m_nurbs.CVCount() - 2);

	//ON_4dPoint cp;
	//fit.m_nurbs.GetCV(fit.m_nurbs.CVCount() - 2, cp);
	//cp.x *= 100;
	//cp.y *= 100;
	//cp.z *= 100;
	//cp.w *= 100;
	//fit.m_nurbs.SetCV(fit.m_nurbs.CVCount() - 2, cp); 

	//fit.m_nurbs.GetCV(fit.m_nurbs.CVCount() - 1, cp);
	//cp.x *= 100;
	//cp.y *= 100;
	//cp.z *= 100;
	//cp.w *= 100;
	//fit.m_nurbs.SetCV(fit.m_nurbs.CVCount() - 1, cp);

	//for (int i = 0; i < fit.m_nurbs.CVCount(); i++)
	//{
	//	cout << fit.m_nurbs.Weight(i) << "  ";
	//}
	//cout << endl;
	//for (int i = 0; i < fit.m_nurbs.CVCount(); i++)
	//{
	//	ON_3dPoint cp;
	//	fit.m_nurbs.GetCV(i, cp);
	//	cout << cp.x << "  ";
	//}
	//cout << endl;


	//VisualizeCurve(fit.m_nurbs, cloud);
	//Tool::showComparison(cloud, nurbsLength_CP_ration_NonUniform(fit.m_nurbs));
	//Tool::showComparison(cloud, nurbsCurve(fit.m_nurbs), 2, 1);
	//Tool::showComparison(cloud, getNurbsCloud(fit.m_nurbs), 2, 1);
	//Tool::showComparison(cloud, nurbsCurve(fit.m_nurbs), getNurbsCloud(fit.m_nurbs));
	//Tool::showComparison(getCurve(controlMatrix), nurbsCurve(fit.m_nurbs), getNurbsCloud(fit.m_nurbs));

	
	//恢复样条到原始点云中
	*cloud = *getCurve(controlMatrix);
	cloud->getMatrixXfMap(3, 4, 0).row(2).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
	cloud->getMatrixXfMap(3, 4, 0).row(1).setConstant(axis_z);
	if (along_y)
	{
		cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
	}
	#endif
	

	return curveLength(controlMatrix);
}

//直线度
double straightness(CP cloud, int empty_dim)
{
	if (empty_dim == 1)
	{
		cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));
	}
	else if (empty_dim == 0)
	{
		cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
		cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));
	}
	else
	{
		throw logic_error("No implement");
	}	

	double mean = cloud->getMatrixXfMap(3, 4, 0).row(0).mean();
	double max_x = cloud->getMatrixXfMap(3, 4, 0).row(0).maxCoeff(); // 找到最大值
	double min_x = cloud->getMatrixXfMap(3, 4, 0).row(0).minCoeff(); // 找到最小值
	cloud->getMatrixXfMap(3, 4, 0).row(0) = cloud->getMatrixXfMap(3, 4, 0).row(0).array() - cloud->getMatrixXfMap(3, 4, 0).row(0).mean();

	double x2 = cloud->getMatrixXfMap(3, 4, 0).row(0).array().square().sum();
	double y = cloud->getMatrixXfMap(3, 4, 0).row(1).array().sum();
	double x = cloud->getMatrixXfMap(3, 4, 0).row(0).array().sum();
	double xy = (cloud->getMatrixXfMap(3, 4, 0).row(0).array() * cloud->getMatrixXfMap(3, 4, 0).row(1).array()).sum();
	int n = cloud->size();

	double a = (n * xy - x * y) / (n * x2 - x*x);
	double b = (x2 * y - x * xy) / (n * x2 - x*x);

	Eigen::VectorXf bias = (cloud->getMatrixXfMap(3, 4, 0).row(0).array() * a - cloud->getMatrixXfMap(3, 4, 0).row(1).array()) + b;

	bias = bias.array() / sqrt(a * a + 1);
	
	VectorXf::Index maxIndex, minIndex;


	double max_value = bias.maxCoeff(&maxIndex); // 找到最大值
	double min_value = bias.minCoeff(&minIndex); // 找到最小值
	double st = max_value - min_value;


	//cout << "K:" << a << endl;
	//cout << "b:" << b << endl;

	//cout << "最大误差：" << max_value << endl;
	//cout << "最小误差" << min_value << endl;

	//cout << "总误差：" << max_value - min_value << endl;

	//Eigen::VectorXf bias2 = cloud->getMatrixXfMap(3, 4, 0).row(1).array() - cloud->getMatrixXfMap(3, 4, 0).row(1).mean();
	//double max_value2 = bias2.maxCoeff(); // 找到最大值
	//double min_value2 = bias2.minCoeff(); // 找到最小值
	//double st2 = max_value2 - min_value2;

	//if (st > 0.1)
	{
		CP generated(new CloudT);

		//for (int x = 765; x < 229428; x++)
		for (double x = min_x - mean - 1; x < max_x - mean + 1; x += 0.1)
		{
			double y_pre = a * x + b;
			PointT p(x, y_pre, 0);
			generated->push_back(p);
		}

		CP compare(new CloudT);
		compare->push_back(cloud->points[maxIndex]);
		compare->push_back(cloud->points[minIndex]);
		cout << "直线度：" << max_value - min_value << endl;
		//cout << "直线度2：" << st2 << endl;

		//Tool::showComparison(cloud, generated, compare, 2, 2, 10);
	}
	//cout << "直线度2：" << st2 << endl;

	return st;
}

//int main()
//{
//	CP cloud(new CloudT);
//
//	pcl::io::loadPCDFile("234.pcd", *cloud);
//
//	#pragma omp parallel for
//	for(int step = 0; step<8; step++)
//	{
//		float y_d = 0.4 + step * 0.04;
//		// 创建条件滤波器
//		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//		pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
//		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 0.49)));
//		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.51)));
//
//		condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y_d)));
//		condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y_d+0.02)));
//
//		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 6.63)));
//		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 162.998)));
//		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 10)));
//		condrem.setCondition(condition);
//		condrem.setInputCloud(cloud);
//
//		CP tmp(new CloudT);
//		// 应用条件滤波
//		condrem.filter(*tmp);
//		
//
//		{
//			pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//			pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
//			condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 12)));
//			condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 6.64)));
//			condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 163.011)));
//
//			//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 18)));
//			//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0)));
//			//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 168)));
//
//			condrem.setCondition(condition);
//			condrem.setInputCloud(tmp);
//
//			// 应用条件滤波
//			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
//			condrem.filter(*filtered);
//
//			//Tool::showComparison(tmp, filtered);
//			verify(filtered, 150, 15, false);
//			//Tool::showComparison(tmp, filtered);
//			BSplineLength bsp;
//			auto ret = bsp.fitBSpline(filtered, false);
//
//
//			cout <<"长:"<< ret[0] << endl;
//			cout << "轮廓度:" << ret[2] << endl;
//
//		}
//	}
//}


////single
//int main()
//{
//	CP cloud(new CloudT);
//
//	pcl::io::loadPCDFile("test_1 - Cloud.pcd", *cloud);
//	//pcl::io::loadPCDFile("gg2.pcd", *cloud);
//
//	straightness(cloud, 1);
//
//	return 0;
//}


//int main()
//{
//	#ifdef CZX_TEST
//	CP cloud(new CloudT);
//	
//	vector<string> path_list;
//	{
//		string root = conf["root"];
//		string inPath = root + "*.pcd";
//		__int64 handle;
//		struct _finddata_t fileinfo;
//		handle = _findfirst(inPath.c_str(), &fileinfo);
//		if (handle == -1)
//			return -1;
//
//		do
//		{
//			string szPath;
//			szPath = root + string(fileinfo.name);
//			path_list.push_back(szPath);
//		} while (!_findnext(handle, &fileinfo));
//	}
//
//	vector<MatrixXd> arcs;
//	vector<MatrixXd> twods;
//
//	Eigen::Tensor<double, 3> arcs(path_list.size(), 2, 9);
//	Eigen::Tensor<double, 3> twods(path_list.size(), 2, 9);
//
//
//	HMODULE hDLL = LoadLibrary(L"curveLength_DLL.dll");
//	typedef vector<vector<vector<double>>>(__cdecl* VVVCVV)(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<CP> fitted_x, vector<CP> fitted_y);
//	VVVCVV getLength = (VVVCVV)GetProcAddress(hDLL, "getLength");
//	if (!getLength)
//		return -1;
//	for (int i_path = 0; i_path < path_list.size(); i_path++) {
//		string path = path_list[i_path];
//		if (pcl::io::loadPCDFile(path, *cloud) == -1)
//		{
//			cout << "读取点云失败" << endl;
//			return -1;
//		}
//		vector<CP> fitted_x, fitted_y;
//		vector<vector<vector<double>>> ret = getLength(cloud, fitted_x, fitted_y);
//		Eigen::MatrixXd cloud_arc(ret.size(), ret[0].size());
//		Eigen::MatrixXd cloud_2d(ret.size(), ret[0].size());
//
//		for (int i=0; i<ret.size();i++)
//		{
//			for(int j=0; j<ret[0].size();j++)
//			{
//				CzxLog("log.czx") << "弧长是：" << ret[i][j][0] << endl;
//				CzxLog("log.czx") << "2D长度是：" << ret[i][j][1] << endl;
//				cloud_arc(i, j) = ret[i][j][0];
//				cloud_2d(i, j) = ret[i][j][1];
//			}
//		}
//		arcs.chip(Index(i_path), 0) = Eigen::TensorMap<Eigen::Tensor<double, 2>>(cloud_arc.data(), cloud_arc.rows(), cloud_arc.cols());
//		twods.chip(Index(i_path), 0) = Eigen::TensorMap<Eigen::Tensor<double, 2>>(cloud_2d.data(), cloud_2d.rows(), cloud_2d.cols());
//		{
//			Eigen::VectorXd max_values = cloud_arc.rowwise().maxCoeff();
//			CzxLog("log.czx") << "弧长最大值：" << std::endl << max_values << std::endl;
//
//			Eigen::VectorXd min_values = cloud_arc.rowwise().minCoeff();
//			CzxLog("log.czx") << "弧长最小值：" << std::endl << min_values << std::endl;
//
//			Eigen::VectorXd mean = cloud_arc.rowwise().mean();
//			Eigen::VectorXd variance = (cloud_arc.colwise() - mean).array().square().rowwise().sum() / (cloud_arc.cols() - 1);
//			CzxLog("log.czx") << "弧长方差：" << std::endl << variance << std::endl;
//			CzxLog("log.czx") << endl;
//		}
//
//		{
//			Eigen::VectorXd max_values = cloud_2d.rowwise().maxCoeff();
//			CzxLog("log.czx") << "2D长度最大值：" << std::endl << max_values << std::endl;
//
//			Eigen::VectorXd min_values = cloud_2d.rowwise().minCoeff();
//			CzxLog("log.czx") << "2D长度最小值：" << std::endl << min_values << std::endl;
//
//			Eigen::VectorXd mean = cloud_2d.rowwise().mean();
//			Eigen::VectorXd variance = (cloud_2d.colwise() - mean).array().square().rowwise().sum() / (cloud_2d.cols() - 1);
//			CzxLog("log.czx") << "2D长度方差：" << std::endl << variance << std::endl;
//			CzxLog("log.czx") << endl;
//		}
//	}
//
//
//
//	for (int i = 0; i < arcs.dimension(1); i++)
//	{
//		for (int j = 0; j < arcs.dimension(2); j++)
//		{
//			{
//				Eigen::Tensor<double, 1> arc_var_tensor = arcs.chip(Index(j), 2).chip(Index(i), 1);
//				Eigen::VectorXd arc_var = Eigen::Map<Eigen::VectorXd>(arc_var_tensor.data(), arc_var_tensor.size());
//				 计算最大值和最小值
//				double max_value = arc_var.maxCoeff();
//				double min_value = arc_var.minCoeff();
//
//				 计算方差
//				double mean = arc_var.mean();
//				double variance = (arc_var.array() - mean).square().sum() / (arc_var.size() - 1);
//
//				 打印结果
//				CzxLog("log.czx") << "云间定域弧长的最大值: " << max_value << std::endl;
//				CzxLog("log.czx") << "云间定域弧长的最小值: " << min_value << std::endl;
//				CzxLog("log.czx") << "云间定域弧长的方差: " << variance << std::endl;
//				CzxLog("log.czx") << endl;
//			}
//
//			{
//				Eigen::Tensor<double, 1> len_var_tensor = twods.chip(Index(j), 2).chip(Index(i), 1);
//				Eigen::VectorXd len_var = Eigen::Map<Eigen::VectorXd>(len_var_tensor.data(), len_var_tensor.size());
//				 计算最大值和最小值
//				double max_value = len_var.maxCoeff();
//				double min_value = len_var.minCoeff();
//
//				 计算方差
//				double mean = len_var.mean();
//				double variance = (len_var.array() - mean).square().sum() / (len_var.size() - 1);
//
//				 打印结果
//				CzxLog("log.czx") << "云间定域弧长的最大值: " << max_value << std::endl;
//				CzxLog("log.czx") << "云间定域弧长的最小值: " << min_value << std::endl;
//				CzxLog("log.czx") << "云间定域弧长的方差: " << variance << std::endl;
//				CzxLog("log.czx") << endl;
//			}
//		}
//	}
//	return 0;
//	#endif
//
//	#ifndef CZX_TEST
//	const string log_path = "log.czx";
//	CzxTimer::path = log_path;
//	vector<CP> stripes_y;
//	vector<CP> stripes_x;
//
//	#ifndef LOAD
//	CP cloud(new CloudT);
//	if (pcl::io::loadPCDFile(conf["cloud_path"], *cloud) == -1)
//	{
//		cout << "读取点云失败" << endl;
//		return -1;
//	}
//
//	modelNormalize(cloud);
//
//	#pragma omp parallel sections
//	{
//		#pragma omp section	
//		stripes_y = process(cloud, true);
//		#pragma omp section
//		stripes_x = process(cloud, false);
//	}
//	#endif
//
//	#ifdef SAVE
//	for (int i = 0; i < stripes_y.size(); i++)
//	{
//		boost::filesystem::create_directory("y");
//		pcl::io::savePCDFileBinary("y/s" + to_string(i) + ".pcd", *stripes_y[i]);
//	}
//	for (int i = 0; i < stripes_x.size(); i++)
//	{
//		boost::filesystem::create_directory("x");
//		pcl::io::savePCDFileBinary("x/s" + to_string(i) + ".pcd", *stripes_x[i]);
//	}
//	#endif
//	
//	CP cloud(new CloudT);
//	pcl::io::loadPCDFile("2.pcd", *cloud);
//	BSplineLength bsp;
//	auto ret = bsp.process(cloud);
//	for (auto stripes : ret)
//	{
//		for (int i = 0; i < stripes.size(); i += 4)
//		{
//			cout << "弧长是：" << stripes[i][0] << endl;
//			cout << "2D长度是：" << stripes[i][1] << endl;
//			cout << "轮廓度是：" << stripes[i][2] << endl;
//			cout << endl;
//		}
//	}
//	return 0;
//
//	#ifdef LOAD
//	for (int i = 0; i < stoi(conf["region_num"]); i++)
//	{
//		CP cloud_y(new CloudT);
//		if(pcl::io::loadPCDFile("y/s" + to_string(i) + ".pcd", *cloud_y)!=-1)
//			stripes_y.push_back(cloud_y);
//		CP cloud_x(new CloudT);
//		if(pcl::io::loadPCDFile("x/s" + to_string(i) + ".pcd", *cloud_x)!=-1)
//			stripes_x.push_back(cloud_x);
//	}
//	#endif
//
//	int repeat_num = stoi(conf["repeat_num"]);
//
//	#pragma omp parallel for
//	for (int i = 0; i < stripes_x.size(); i++)
//	{
//		for (int _ = 0; _ < repeat_num; _++)
//		{
//			CP tmp(new CloudT);
//			*tmp = *stripes_x[i];
//			BSplineLength bsp;
//			cout << bsp.fitBSpline(tmp, false)[0] << endl;
//			#ifdef SAVE
//			pcl::io::savePCDFileBinary("x/fitted" + to_string(i) + ".pcd", *tmp);
//			#endif
//		}
//	}
//
//	#pragma omp parallel for
//	for (int i = 0; i < stripes_y.size(); i++)
//	{
//		for (int _ = 0; _ < repeat_num; _++)
//		{
//			CP tmp(new CloudT);
//			*tmp = *stripes_y[i];
//			BSplineLength bsp;
//			cout << bsp.fitBSpline(tmp, true)[0] << endl;
//			#ifdef SAVE
//			pcl::io::savePCDFileBinary("y/fitted" + to_string(i) + ".pcd", *tmp);
//			#endif
//		}
//	}
//	//cout << fitBSpline(stripes_y[3], true) << endl;
//	#endif
//	return 0;
//}

double getAngle(CP cloud, int index, int range)
{
	CP c1(new CloudT);
	c1->resize(range);
	c1->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, index, 3, range);
	pcl::PCA<PointT> pca;
	pca.setInputCloud(c1);
	Vector3f v1 = pca.getEigenVectors().col(0);


	CP c2(new CloudT);
	c2->resize(range);
	c2->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, index-range+1, 3, range);
	pca.setInputCloud(c2);
	Vector3f v2 = pca.getEigenVectors().col(0);
	float angle = pcl::getAngle3D(v1, v2, true);
	if (angle > 90)
		angle = 180 - angle;

	//Tool::showComparison(c1, c2, cloud, 5, 5, 2);

	return angle;
}

double normalAngele(CP cloud, int index)
{
	CP c1(new CloudT);
	int size = 11;
	c1->resize(size);
	c1->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, index-(size-1)/2, 3, size);
	pcl::PCA<PointT> pca;
	pca.setInputCloud(c1);
	Vector3f v1 = pca.getEigenVectors().col(1);

	CP c2(new CloudT);
	c2->resize(size);
	c2->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, index-(size-1)/2 + 1, 3, size);
	pca.setInputCloud(c2);
	Vector3f v2 = pca.getEigenVectors().col(1);

	float angle = pcl::getAngle3D(v1, v2, true);
	if (angle > 90)
		angle = 180 - angle;

	//cout << angle << endl;
	//Tool::showComparison(c1, c2, cloud, 5, 3, 2);
	return angle;
}

//滤掉楞点云
void filter(CP cloud)
{
	int cur_index = cloud->size()-100;
	for (; cur_index < cloud->size()-1; cur_index++)
	{
		PointT cur = cloud->at(cur_index);
		PointT next = cloud->at(cur_index+1);
		if (cur.y - next.y > 0.05)
		{
			break;
		}
	}

	for (cur_index--; cur_index > 1; cur_index--)
	{
		PointT last = cloud->at(cur_index-1);
		PointT cur = cloud->at(cur_index);
		PointT next = cloud->at(cur_index + 1);
		Vector3f p1 = last.getArray3fMap();
		Vector3f p2 = cur.getArray3fMap();
		Vector3f p3 = next.getArray3fMap();
		Vector3f v1 = p2 - p1;
		Vector3f v2 = p3 - p2;

		//float angle = getAngle(cloud, cur_index, 3);

		float angle = pcl::getAngle3D(v1, v2, true);
		if (angle > 30)
			break;
	}

	cloud->erase(cloud->begin() + cur_index, cloud->end());


	cur_index = 0;
	bool found = false;
	for (; cur_index < cloud->size() - 1; cur_index++)
	{
		PointT cur = cloud->at(cur_index);
		PointT next = cloud->at(cur_index+1);
		if (!found)
		{
			if (abs(cur.y - next.y) > 0.05)
			{
				found=true;
			}
		}
		else
		{
			if (abs(cur.y - next.y) < 0.05)
			{
				cur_index++;
				break;
			}
		}
	}

	if (cur_index == cloud->size() - 1)
	{
		cur_index = 1;
	}

	for (; cur_index < cloud->size()-1; cur_index++)
	{
		PointT last = cloud->at(cur_index-1);
		PointT cur = cloud->at(cur_index);
		PointT next = cloud->at(cur_index+1);
		Vector3f p1 = last.getArray3fMap();
		Vector3f p2 = cur.getArray3fMap();
		Vector3f p3 = next.getArray3fMap();
		Vector3f v1 = p2 - p1;
		Vector3f v2 = p3 - p2;

		float angle = pcl::getAngle3D(v1, v2, true);
		if (angle > stof(conf["min_angle"]))
		{
			volatile float x = angle;
			break;
		}
	}
	cloud->erase(cloud->begin(), cloud->begin()+ cur_index);
}


////低通点云的滤波
//void filter(CP cloud)
//{
//	int cur_index = 5;
//	int range = stoi(conf["range"]);
//	for (; cur_index < cloud->size()-1; cur_index++)
//	{
//		PointT last = cloud->at(cur_index-1);
//		PointT cur = cloud->at(cur_index);
//		PointT next = cloud->at(cur_index + 1);
//		Vector3f p1 = last.getArray3fMap();
//		Vector3f p2 = cur.getArray3fMap();
//		Vector3f p3 = next.getArray3fMap();
//		Vector3f v1 = p2 - p1;
//		Vector3f v2 = p3 - p2;
//
//		float angle = getAngle(cloud, cur_index, range);
//
//		//float angle = pcl::getAngle3D(v1, v2, true);
//		if (angle > stof(conf["min_angle"]))
//			break;
//	}
//	
//	cloud->erase(cloud->begin(), cloud->begin() + cur_index);
//}


////法线夹角滤波
//void filter(CP cloud)
//{
//	int cur_index = 5;
//	int range = stoi(conf["range"]);
//	float last_normal_angle = 999;
//	for (; cur_index < cloud->size() - 1; cur_index++)
//	{
//		PointT last = cloud->at(cur_index - 1);
//		PointT cur = cloud->at(cur_index);
//		PointT next = cloud->at(cur_index + 1);
//		Vector3f p1 = last.getArray3fMap();
//		Vector3f p2 = cur.getArray3fMap();
//		Vector3f p3 = next.getArray3fMap();
//		Vector3f v1 = p2 - p1;
//		Vector3f v2 = p3 - p2;
//
//
//
//		float normal_angle = normalAngele(cloud, cur_index);
//		if (last_normal_angle > normal_angle && normal_angle > 3)
//		{
//			break;
//		}
//		last_normal_angle = normal_angle;
//		//float angle = getAngle(cloud, cur_index, range);
//
//		//float angle = pcl::getAngle3D(v1, v2, true);
//		//if (angle < stof(conf["min_angle"]))
//		//	continue;		
//		//float normal_angle = normalAngele(cloud, cur_index);
//		//if (normal_angle < 1)
//		//	break;
//
//	}
//
//	cloud->erase(cloud->begin(), cloud->begin() + cur_index);
//}


////夹角
//void filter(CP cloud)
//{
//	//cloud->erase(cloud->begin() + 40, cloud->end());
//	int cur_index = 1;
//	for (; cur_index < cloud->size() - 1; cur_index++)
//	{
//		PointT last = cloud->at(cur_index - 1);
//		PointT cur = cloud->at(cur_index);
//		Vector3f p1 = last.getArray3fMap();
//		Vector3f p2 = cur.getArray3fMap();
//		Vector3f v1 = p2 - p1;
//		Vector3f v2(1.0, 0, 0);
//		
//		float angle = pcl::getAngle3D(v1, v2, true);
//		if (angle < 56)
//			break;
//		//cout << angle<<endl;
//		//CP c1(new CloudT);
//		//c1->push_back(last);
//		//c1->push_back(cur);
//		//Tool::showComparison(c1, cloud, 5, 2);
//	}
//	cloud->erase(cloud->begin(), cloud->begin() + cur_index);
//}

//int main()
//{
//	vector<string> path_list;
//	{
//		string root = conf["root"];
//		string inPath = root + "*.pcd";
//		__int64 handle;
//		struct _finddata_t fileinfo;
//		handle = _findfirst(inPath.c_str(), &fileinfo);
//		if (handle == -1)
//			return -1;
//
//		do
//		{
//			//找到的文件的文件名
//			string szPath;
//			szPath = root + string(fileinfo.name);
//			path_list.push_back(szPath);
//		} while (!_findnext(handle, &fileinfo));
//	}
//
//	bool along_y;
//	along_y = conf["along_y"] == "true";
//
//	////#pragma omp parallel for
//	//for (int i = 0; i < path_list.size(); i++)
//	//{
//	//	CP cloud(new CloudT);
//	//	pcl::io::loadPCDFile(path_list[i], *cloud);
//	//	cloud->getMatrixXfMap(3, 4, 0).row(2) /= 1000;
//	//	verify(cloud, 0, 100, false);
//	//	filter(cloud, stof(conf["min_angle"]), false);
//	//	cout << "直线度:" << straightness(cloud, 1) << endl;
//	//}
//
//
//	#pragma omp parallel for
//	for (int i = 0; i < path_list.size(); i++)
//	{
//		CP cloud(new CloudT);
//
//		//pcl::io::loadPCDFile(conf["cloud_path"], *cloud);
//		pcl::io::loadPCDFile(path_list[i], *cloud);		
//
//		//arsenal::fourierTransform(cloud);
//
//		//cloud->getMatrixXfMap(3, 4, 0).row(0) /= 250;
//		//cloud->getMatrixXfMap(3, 4, 0).row(0) /= 250;
//		//if (cloud->points[0].y == 0.0)
//		//{
//		//	cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
//		//}
//		//pcl::io::savePCDFile(path_list[i], *cloud);
//
//		pcl::PassThrough<PointT> pass;
//		pass.setInputCloud(cloud);
//		pass.setFilterFieldName("y");
//		pass.setFilterLimits(-2, FLT_MAX);
//		pass.filter(*cloud);
//
//		//{
//		//	CzxComparison fdghbajb(cloud);
//		//	filter(cloud);
//		//}
//
//		BSplineLength bsp;
//		bsp.conf=czxTool::readProfile("conf.czxconf");
//
//		//verify(cloud, 0, 100, along_y);
//		//bool filter_success = filter(cloud, stof(conf["min_angle"]), along_y);
//		//cout << filter_success << endl;
//		//cout << fitBSpline(cloud, along_y) << endl;
//
//		{
//			//CzxComparison fdghbajb(cloud);
//			auto ret = bsp.fitBSpline(cloud);
//
//			//if (ret[0] < 100)
//			//	Tool::show(cloud);
//			cout << "长:" << ret[0] << endl;
//			cout << "轮廓度:" << ret[2] << endl;
//		}
//	}
//}

int main()
{
	CP cloud(new CloudT);
	pcl::io::loadPCDFile("2.pcd", *cloud);
	//Tool::show(cloud);
	{
		//CP copy(new CloudT);
		//CP ret(new CloudT);
		//*copy = *cloud;
		//*cloud = *copy;
		//arsenal::passThrough(cloud, "x", -82.5, -82.4);
		//*ret += *cloud;
		//*cloud = *copy;
		//arsenal::passThrough(cloud, "x", -55.22, -55.0);
		//*ret += *cloud;
		//*cloud = *copy;
		//arsenal::passThrough(cloud, "x", -28.42, -28.20);
		//*ret += *cloud;
		//*cloud = *copy;
		//arsenal::passThrough(cloud, "x", -2.57, -2.35);
		//*ret += *cloud;
		//Tool::showComparison(copy, ret);
		arsenal::passThrough(cloud, "x", -2.57, -2.55);
		sort(cloud->begin(), cloud->end(), [](PointT a, PointT b) { return a.x > b.x;});
	}
	cloud->getMatrixXfMap(3, 4, 0).row(0).setZero();
	cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
	cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));

	//BSplineLength bsp;
	//auto ret = bsp.fitBSpline(cloud);
	HMODULE hDLL = LoadLibrary(L"czxToolkit.dll");
	typedef vector<float>(__cdecl* MYTYPE)(CP cloud, bool visual);
	MYTYPE fitBSpline_ = (MYTYPE)GetProcAddress(hDLL, "fitBSpline_");
	auto ret = fitBSpline_(cloud, false);
	//{
	//	pcl::visualization::PCLVisualizer viewer;
	//	viewer.addPointCloud(getNurbsCloud(bsp.nurbs_curve));
	//	viewer.addPointCloud(cloud);
	//	viewer.addText(to_string(ret[0]), 100, 100);
	//	viewer.spin();
	//}
}