#include "BSplineLength.h"

//#define CZX_DEBUG

BSplineLength::BSplineLength(string conf_path)
{
	try
	{
		conf = czxTool::readProfile(conf_path);
	}
	catch (exception e)
	{
	}
	visual = false;
}


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
bool BSplineLength::verify(CP cloud, float min_length, float min_points, bool along_y)
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

vector<CP> BSplineLength::slice(const CP cloud, bool along_y)
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

	for (int star = 0; star < steps; star++)
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

		if (along_y)
			qualified = verify(stripe, 0.95 * (maxPoint.y - minPoint.y), 100, along_y);
		else
			qualified = verify(stripe, 0.95 * (maxPoint.x - minPoint.x), 100, along_y);
		if (!qualified) continue;

		ret.push_back(stripe);
	}
	return ret;
}

vector<CP> BSplineLength::choice(vector<CP> clouds, int num)
{
	sort(clouds.begin(), clouds.end(), [](CP first, CP second) {return first->size() > second->size(); });
	if (clouds.size() < num) num = clouds.size();
	return vector<CP>(clouds.begin(), clouds.begin() + num);
}

Eigen::Matrix3f BSplineLength::modelNormalize(CP cloud)
{
	CzxTimer fbhgdjusfgvuw(__func__);
	pcl::PCA<PointT> pca;
	pca.setInputCloud(cloud);
	Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
	preProcess pp;
	CP keypoints = pp.process(cloud);
	cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);
	if (pp.state)
	{
		keypoints->getMatrixXfMap(3, 4, 0) = projection * keypoints->getMatrixXfMap(3, 4, 0);
		MatrixXf keypoints_2d_eigen = keypoints->getMatrixXfMap(3, 4, 0).block(0, 0, 2, 3);
		VectorXf x_axis = keypoints_2d_eigen.col(1) - keypoints_2d_eigen.col(0);
		x_axis.normalize();
		VectorXf y_axis(2);
		y_axis << -x_axis[1], x_axis[0];

		Matrix3f projection_xy = Matrix3f::Identity();
		projection_xy.block(0, 0, 1, 2) = x_axis.transpose();
		projection_xy.block(1, 0, 1, 2) = y_axis.transpose();
		cloud->getMatrixXfMap(3, 4, 0) = projection_xy * cloud->getMatrixXfMap(3, 4, 0);
		projection = projection_xy * projection;
	}
	#ifdef CZX_DEBUG
	//pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
	#endif
	return projection;
}

vector<CP> BSplineLength::getStripes(CP cloud, bool along_y)
{
	vector<CP> stripes;
	PointT minPoint;
	PointT maxPoint;
	float step;
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);

	int region_num = stoi(conf["region_num"]);
	if (along_y)
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

Vector2d BSplineLength::differentialOne(MatrixXd pm, double tf)
{
	MatrixXd cm(4, 4);
	cm << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 0, 3, 0,
		1, 4, 1, 0;
	MatrixXd  tm_diff(4, 1);
	tm_diff << 3 * tf * tf, 2 * tf, 1, 0;
	Vector2d differential = pm * cm.transpose() * tm_diff / 6;
	return differential;
}

double BSplineLength::curveLength(Eigen::MatrixXd controlMatrix)
{
	int cols = controlMatrix.cols();
	if (cols <= 4)
		return numeric_limits<double>::quiet_NaN();

	double length = 0;
	double interval = stod(conf["interval"]);

	//#pragma omp parallel for
	for (int i = 0; i <= cols - 4; i++)
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

Vector2d BSplineLength::getPos(MatrixXd pm, double tf)
{
	MatrixXd cm(4, 4);
	cm << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 0, 3, 0,
		1, 4, 1, 0;
	MatrixXd  tm_diff(4, 1);
	tm_diff << tf * tf * tf, tf* tf, tf, 1;
	Vector2d pos = pm * cm.transpose() * tm_diff / 6;
	return pos;
}

double BSplineLength::curveLengthTest(Eigen::MatrixXd controlMatrix)
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

vector<double> BSplineLength::fitBSpline(CP cloud, bool along_y)
{
	CzxTimer sdhfadh(__func__);
	unsigned order(4);		//阶数
	unsigned n_control_points(cloud->size() / stoi(conf["controlDense"]) + 4);		//控制点个数

	bool (*compare)(PointT, PointT);
	if (along_y)
		compare = [](PointT first, PointT second) {return first.y < second.y; };
	else
		compare = [](PointT first, PointT second) {return first.x < second.x; };

	sort(cloud->begin(), cloud->end(), compare);

	if (along_y)
	{
		cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1)); //y轴向x轴迁移
	}
	cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));
	//#ifdef CZX_DEBUG
	float axis_z = cloud->getMatrixXfMap(3, 4, 0).row(2).mean();
	//#endif
	cloud->getMatrixXfMap(3, 4, 0).row(2).setZero();

	auto ret = fitBSpline(cloud);

	////恢复样条到原始点云中
	cloud->getMatrixXfMap(3, 4, 0).row(2).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
	cloud->getMatrixXfMap(3, 4, 0).row(1).setConstant(axis_z);
	if (along_y)
	{
		cloud->getMatrixXfMap(3, 4, 0).row(0).swap(cloud->getMatrixXfMap(3, 4, 0).row(1));
	}

	return ret;
}

vector<double> BSplineLength::fitBSpline(CP cloud)
{
	CzxTimer sdhfadh(__func__);
	unsigned order(4);		//阶数
	int n_control_points(cloud->size() / stoi(conf["controlDense"]) + 4);		//控制点个数
	//int n_control_points=100;		//控制点个数
	n_control_points = std::max(n_control_points, 100);

	CP ret(new CloudT);

	pcl::on_nurbs::NurbsDataCurve2d data;

	PointCloud2Vector2d(cloud, data.interior);

	pcl::on_nurbs::FittingCurve2d::Parameter curve_params;
	curve_params.smoothness = 0.01;		//光滑度
	curve_params.rScale = 0.02;		//尺度


	ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2d::initNurbsPCA(order, &data, n_control_points);
	curve.MakeRational();
	pcl::on_nurbs::FittingCurve2d fit(&data, curve);
	{
		CzxTimer _("assemble");
		fit.assemble(curve_params);		//装配曲线
	}

	//设置固定点
	Eigen::Vector2d star(cloud->points[0].x, cloud->points[0].y);
	Eigen::Vector2d end(cloud->points[cloud->size() - 1].x, cloud->points[cloud->size() - 1].y);
	if (star[0] > end[0])
	{
		swap(star, end);
	}
	fit.addControlPointConstraint(0, star, 100);
	fit.addControlPointConstraint(curve.CVCount() - 1, end, 100); // 小一点的权重不一定会让最终控制点是首尾点	
	{
		CzxTimer sdhfadh1("solve");
		fit.solve();
	}
	double line_profile = lineProfile(fit, cloud);

	double cur_len = curveLengthPcl(fit.m_nurbs);
	PointT minPoint;
	PointT maxPoint;
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);

	float dif;
	dif = maxPoint.x - minPoint.x;
	{
		#ifdef CZX_DEBUG
		//Tool::showComparison(cloud, getNurbsCloud(fit.m_nurbs), 3, 1);
		CP fitted_curve = getNurbsCloud(fit.m_nurbs);
		double max_angle = -1;
		for (int index = 0; index < fitted_curve->size() - 2; index++)
		{
			Vector3f v1 = fitted_curve->points[index].getVector3fMap() - fitted_curve->points[index + 1].getVector3fMap();
			Vector3f v2 = fitted_curve->points[index + 1].getVector3fMap() - fitted_curve->points[index  + 2].getVector3fMap();
			auto angle = pcl::getAngle3D(v1, v2, true);
			angle = abs(angle);
			if (angle > 90) 
				angle = 180 - angle;
			max_angle = max(angle, max_angle);
		}
		//pcl::visualization::PCLVisualizer viewer;
		//viewer.addPointCloud(cloud, "1");
		//viewer.addPointCloud(fitted_curve, "2");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "1");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "2");
		//viewer.addText(to_string(max_angle) + " ", 400, 800, 50, 0, 1, 0, "text");
		//viewer.spin();

		#endif
		*cloud = *getNurbsCloud(fit.m_nurbs);
	}
	nurbs_curve = fit.m_nurbs;
	return vector<double>({ cur_len, dif, line_profile });
}

void BSplineLength::PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		pcl::PointXYZ& p = cloud->at(i);
		if (!std::isnan(p.x) && !std::isnan(p.y))
			data.push_back(Eigen::Vector2d(p.x, p.y));
	}
}

ON_3dPoint BSplineLength::inverseMapping(pcl::KdTreeFLANN<PointT>& nurbs_tree, CP cloud, ON_3dPoint target)
{
	std::vector<int> pointIdxNKNSearch; // 用于存储最近邻点的索引
	std::vector<float> pointNKNSquaredDistance; // 用于存储最近邻点的距离的平方
	if (nurbs_tree.nearestKSearch(PointT(target.x, target.y, target.z), 2, pointIdxNKNSearch, pointNKNSquaredDistance) < 2)
		throw logic_error("找不到近邻");
	return ON_3dPoint(cloud->points[pointIdxNKNSearch[1]].x, cloud->points[pointIdxNKNSearch[1]].y, cloud->points[pointIdxNKNSearch[1]].z);
}

double BSplineLength::lineProfile(const pcl::on_nurbs::FittingCurve2d& fit, CP cloud)
{
	CzxTimer gdgag(__func__);
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
	if (visual)
	{
		cout << "轮廓度" << 2 * max_error << endl;
		ON_3dPoint target(fit.m_data->interior[arg_max_error][0], fit.m_data->interior[arg_max_error][1], 0);
		ON_3dPoint pt = inverseMapping(curve_kdtree, curve_cloud, target);
		CP maxErrorPoint(new CloudT());
		maxErrorPoint.reset(new CloudT);
		maxErrorPoint->push_back(PointT(fit.m_data->interior[arg_max_error][0], fit.m_data->interior[arg_max_error][1], 0));
		maxErrorPoint->push_back(PointT(pt.x, pt.y, 0));
		Tool::showComparison(cloud, curve_cloud, maxErrorPoint, 3, 1, 5);
	}
	return 2 * max_error;
}

double BSplineLength::lineProfileMy(const pcl::on_nurbs::FittingCurve2d& fit, CP cloud)
{
	CzxTimer gdgag(__func__);
	CP curve_cloud = getNurbsCloudMy(fit.m_nurbs);
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
	if (visual)
	{
		cout << "轮廓度" << 2 * max_error << endl;
		ON_3dPoint target(fit.m_data->interior[arg_max_error][0], fit.m_data->interior[arg_max_error][1], 0);
		ON_3dPoint pt = inverseMapping(curve_kdtree, curve_cloud, target);
		CP maxErrorPoint(new CloudT());
		maxErrorPoint.reset(new CloudT);
		maxErrorPoint->push_back(PointT(fit.m_data->interior[arg_max_error][0], fit.m_data->interior[arg_max_error][1], 0));
		maxErrorPoint->push_back(PointT(pt.x, pt.y, 0));
		Tool::showComparison(cloud, curve_cloud, maxErrorPoint, 3, 1, 5);
	}
	return 2 * max_error;
}

CP BSplineLength::getNurbsCloud(const ON_NurbsCurve& nurbs)
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

CP BSplineLength::getNurbsCloudMy(const ON_NurbsCurve& nurbs)
{
	auto controlMatrix = getControl(nurbs);
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

double BSplineLength::euclideanDistance(const double* array1, const double* array2, int dim) {
	double distance = 0.0;
	for (int i = 0; i < 2; ++i) {
		distance += std::pow(array1[i] - array2[i], 2);
	}

	return std::sqrt(distance);
}

double BSplineLength::curveLengthPcl(const ON_NurbsCurve& nurbs)
{
	CzxTimer safd(__func__);
	double interval = stod(conf["interval"]) / 100;
	double last_point[2];
	double point[2];
	double sum = 0;
	nurbs.Evaluate(0, 0, 2, last_point);
	double last_knot = nurbs.Knot(nurbs.KnotCount() - 1);
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

vector<vector<vector<double>>> BSplineLength::process(CP cloud)
{
	CzxTimer::path = "curveLog.czx";
	vector<vector<vector<double>>> ret;

	modelNormalize(cloud);

	stripes_x = getStripes(cloud, false);
	vector<vector<double>> x_ret;
	for (int i = 0; i < stripes_x.size(); i++)
	{		
		CP tmp(new CloudT);
		*tmp = *stripes_x[i];
		x_ret.push_back(fitBSpline(tmp, false));
		//pcl::io::savePCDFileBinary("x/fitted" + to_string(i) + ".pcd", *tmp);
		fitted_x.push_back(tmp);
	}
	ret.push_back(x_ret);

	stripes_y = getStripes(cloud, true);
	vector<vector<double>> y_ret;
	for (int i = 0; i < stripes_y.size(); i++)
	{
		CP tmp(new CloudT);
		*tmp = *stripes_y[i];
		y_ret.push_back(fitBSpline(tmp, true));
		//pcl::io::savePCDFileBinary("y/fitted" + to_string(i) + ".pcd", *tmp);
		fitted_y.push_back(tmp);
	}
	ret.push_back(y_ret);
	return ret;
}

MatrixXd BSplineLength::getControl(const ON_NurbsCurve& nurbs)
{
	Eigen::MatrixXd controlMatrix(2, nurbs.CVCount() + 4);
	{
		ON_3dPoint cp;
		nurbs.GetCV(0, cp);
		controlMatrix(0, 0) = cp.x;
		controlMatrix(1, 0) = cp.y;
		controlMatrix(0, 1) = cp.x;
		controlMatrix(1, 1) = cp.y;
		for (int i = 0; i < nurbs.CVCount(); i++)
		{
			nurbs.GetCV(i, cp);
			controlMatrix(0, i + 2) = cp.x;
			controlMatrix(1, i + 2) = cp.y;

		}
		nurbs.GetCV(nurbs.CVCount() - 1, cp);
		controlMatrix(0, nurbs.CVCount() + 2) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 2) = cp.y;
		controlMatrix(0, nurbs.CVCount() + 3) = cp.x;
		controlMatrix(1, nurbs.CVCount() + 3) = cp.y;
	}
	return controlMatrix;
}

double BSplineLength::curveLengthGaussian(const ON_NurbsCurve& nurbs)
{
	CzxTimer safd(__func__);
	MatrixXd controls = getControl(nurbs);
	auto fx = [controls](int index, double x) {
		double tf = (x + 1) / 2;

		MatrixXd cm(4, 4);
		cm << -1, 3, -3, 1,
			3, -6, 0, 4,
			-3, 3, 3, 1,
			1, 0, 0, 0;
		MatrixXd  pm(2, 4);
		pm = controls.block(0, index, 2, 4);
		MatrixXd  tm_diff(4, 1);
		tm_diff << 3 * tf * tf, 2 * tf, 1, 0;
		Vector2d diff = pm * cm * tm_diff / 6;
		return diff.norm();
	};

	double ret = 0;
	for (int i = 0; i < controls.cols() - 3; i++)
	{
		double integral =
			0.10122854 * fx(i, 0.96028986) +
			0.10122854 * fx(i, -0.96028986) +
			0.22238103 * fx(i, 0.79666648) +
			0.22238103 * fx(i, -0.79666648) +
			0.31370665 * fx(i, 0.52553241) +
			0.31370665 * fx(i, -0.52553241)+
			0.36268378 * fx(i, 0.18343464) +
			0.36268378 * fx(i, -0.18343464)
			;

		ret += integral/2;
	}

	return ret;
}
