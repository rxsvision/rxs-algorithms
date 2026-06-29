#include "OMPSurface.h"

OMPSurface::OMPSurface(int sqrted_zone_num)
{
	zone_num = sqrted_zone_num * sqrted_zone_num;
	conf = czxTool::readProfile("conf_surface.czx");
}

OMPSurface::~OMPSurface()
{
	for (auto v : sub_clouds)
	{
		delete v.fit;
	}
}

void OMPSurface::setInputCloud(CP cloud)
{
	CzxTimer asdfgbhajsdbgj(__func__);
	int size_sub = sqrt(zone_num);
	main_cloud = cloud;
	sub_clouds.reserve(10000);//预留足够内存避免在多线程环境中vector重新申请内存块
	devideCloud(cloud, size_sub, sub_clouds);
}

//void OMPSurface::setInputCloud(CP cloud, float min_x, float max_x, float min_y, float max_y)
//{
//	CzxTimer asdfgbhajsdbgj(__func__);
//	int size_sub = sqrt(zone_num);
//
//	PointT max_point;
//	//pcl::getMinMax3D(*cloud, min_point, max_point);
//	min_point.x = min_x;
//	min_point.y = min_y;
//
//	max_point.x = max_x;
//	max_point.y = max_y;
//
//	double epsilon = 1e-9;
//
//	dif_x = max_point.x - min_point.x + epsilon;
//	dif_y = max_point.y - min_point.y + epsilon;
//
//	step_x = dif_x / size_sub + epsilon;
//	step_y = dif_y / size_sub + epsilon;
//
//	#pragma omp parallel for ordered
//	for (int ij = 0; ij < zone_num; ij++)
//	{
//		int i = ij / size_sub;
//		int j = ij % size_sub;
//
//		CP sub_cloud(new CloudT);
//		*sub_cloud = *cloud;
//		arsenal::passThrough(sub_cloud, min_point.x + i * step_x, min_point.x + (i + 1) * step_x, min_point.y + j * step_y, min_point.y + (j + 1) * step_y);
//		#pragma omp ordered
//		{
//			sub_clouds.push_back(sub_cloud);
//		}
//	}
//}

bool OMPSurface::devideCloud(CP cloud, int size_sub, VRC& output)
{
	PointT min_point, max_point;
	pcl::getMinMax3D(*cloud, min_point, max_point);

	double epsilon = 1e-6;

	double dif_x = max_point.x - min_point.x + epsilon;
	double dif_y = max_point.y - min_point.y + epsilon;

	double step_x = dif_x / size_sub + epsilon;
	double step_y = dif_y / size_sub + epsilon;

	#pragma omp parallel for
	for (int ij = 0; ij < size_sub* size_sub; ij++)
	{
		int i = ij / size_sub;
		int j = ij % size_sub;

		CP sub_cloud(new CloudT);
		*sub_cloud = *cloud;

		ROICloud tmp;
		tmp.min_x = min_point.x + i * step_x - epsilon;
		tmp.max_x = min_point.x + (i + 1) * step_x + epsilon;
		tmp.min_y = min_point.y + j * step_y - epsilon;
		tmp.max_y = min_point.y + (j + 1) * step_y + epsilon;

		arsenal::passThrough(sub_cloud, tmp.min_x, tmp.max_x, tmp.min_y, tmp.max_y);
		
		if (sub_cloud->size() == 0)
			cout << "error" << endl;


		tmp.cloud = sub_cloud;
		#pragma omp critical
		{
			output.push_back(tmp);
		}
	}
	return true;
}

template <typename T>
void OMPSurface::PointCloud2Vector3d(T cloud, pcl::on_nurbs::vector_vec3d& data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		PointT& p = cloud->at(i);
		if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
			data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

template<typename T>
CP OMPSurface::getNurb(T nurbs)
{
	CP ret(new CloudT);
	double interval_i = 0.001, interval_j = 0.01;
	for (double i = 0; i < 1; i += interval_i)
	{
		for (double j = 0; j < 1; j += interval_j)
		{
			double pnt[3];
			bool success = nurbs.Evaluate(i, j, 0, 3, pnt);
			if (!success)
			{
				cout << "failure" << endl;
			}
			ret->push_back(PointT((float)pnt[0], (float)pnt[1], (float)pnt[2]));
		}
	}
	return ret;
}

template <typename FIT, typename CLOUD> double OMPSurface::surfaceProfile(FIT& fit, CLOUD cloud)
{
	//CzxTimer sdag(__func__);
	double max_dis = 0;
	Vector3d farthest;
	PointT farthest_source;
	for (auto p : *cloud)
	{
		Vector3d target(p.x, p.y, p.z);
		Vector3d cloest;
		Vector2d hint(0.5, 0.5);
		double error;
		Vector3d tu, tv;
		//fit.inverseMapping(fit.m_nurbs, target, hint, cloest, 100, 1e-9, false);
		fit.inverseMapping(fit.m_nurbs, target, hint, error, cloest, tu, tv, 100, 1e-9, true);
		
		
		//double error_c = error;
		//bool flag_changed = false;
		//double best_i, best_j;
		//if(error>0.001 && error > max_dis && error < 0.003)
		//{
		//	double interval_i = 0.0001, interval_j = 0.0001;
		//	for (double i = -0.1; i < 0.1; i += interval_i)
		//	{
		//		for (double j = -0.1; j < 0.1; j += interval_j)
		//		{
		//			double pnt[3];
		//			bool success = fit.m_nurbs.Evaluate(hint[0]+i, hint[1]+j, 0, 3, pnt);
		//			if (!success)
		//			{
		//				cout << "failure" << endl;
		//			}
		//			Vector3d candidate(pnt);
		//			double d = (candidate - target).norm();
		//			if (d - error < -0.001)
		//			{
		//				error = d;
		//				flag_changed = true;
		//				best_i = i;
		//				best_j = j;
		//			}
		//		}
		//	}
		//}
		//if (flag_changed)
		//{
		//	cout << error_c << endl;
		//	cout << error << endl;
		//	cout << best_i << endl;
		//	cout << best_j << endl;
		//}


		if (error > max_dis)
		{
			max_dis = error;
			#ifdef CZX_DEBUG
			farthest = cloest;
			farthest_source = p;
			#endif
		}
	}

	
	//cout << "surface profile: " << max_dis << endl;
	
	//#ifdef CZX_DEBUG
	//CP farthest_point(new CloudT);
	//farthest_point->push_back(PointT(farthest[0], farthest[1], farthest[2]));
	//CP farthest_point_source(new CloudT);
	//farthest_point_source->push_back(farthest_source);
	//if (max_dis > 0.001)
	//	Tool::showComparison(cloud, farthest_point, farthest_point_source, 2, 1, 3);
	//#endif
	return max_dis;
}

template <typename FIT, typename CLOUD> bool OMPSurface::Verify(FIT& fit, CLOUD cloud, double threshold_dis, double threshold_num)
{
	//CzxTimer sdag(__func__);
	int count = 0;
	for (auto p : *cloud)
	{
		Vector3d target(p.x, p.y, p.z);
		Vector3d cloest;
		Vector2d hint(0.5, 0.5);
		double error;
		Vector3d tu, tv;
		//fit.inverseMapping(fit.m_nurbs, target, hint, cloest, 100, 1e-9, false);
		fit.inverseMapping(fit.m_nurbs, target, hint, error, cloest, tu, tv, 100, 1e-9, true);
		if (error > threshold_dis)
		{
			count++;
			if (count > threshold_num)
				return false;
		}
	}


	//cout << "surface profile: " << max_dis << endl;

	//#ifdef CZX_DEBUG
	//CP farthest_point(new CloudT);
	//farthest_point->push_back(PointT(farthest[0], farthest[1], farthest[2]));
	//CP farthest_point_source(new CloudT);
	//farthest_point_source->push_back(farthest_source);
	//if (max_dis > 0.001)
	//	Tool::showComparison(cloud, farthest_point, farthest_point_source, 2, 1, 3);
	//#endif
	//cout << "true" << endl;
	return true;
}

template<typename FIT, typename CLOUD>
CP OMPSurface::reconstruction(FIT& fit, typename CLOUD cloud)
{
	CP ret(new CloudT);
	for (auto p : *cloud)
	{
		Vector3d target(p.x, p.y, p.z);
		Vector3d cloest;
		Vector2d hint(0.5, 0.5);

		double error;
		Vector3d tu, tv;
		fit.inverseMapping(fit.m_nurbs, target, hint, error, cloest, tu, tv, 100, 1e-9, true);

		//#ifdef CZX_DEBUG
		//if ((target - cloest).norm() > 1)
		//{
		//	debug_mtx.lock();
		//	int N = cloud->size() / 3400;
		//	if (N < 1) N = 1;
		//	cout << N << endl;
		//	CP cloud_d = arsenal::downSampleByIndex<CloudT>(cloud, N);
		//	Tool::showComparison(cloud, cloud_d);
		//	CP c1(new CloudT), c2(new CloudT);
		//	c1->push_back(PointT(cloest[0], cloest[1], cloest[2]));
		//	c2->push_back(p);
		//	Tool::showComparison(cloud, c1, c2, 1, 3, 5);
		//	CP nurbs = getNurb(fit.m_nurbs);
		//	Tool::showComparison(cloud, nurbs);
		//	debug_mtx.unlock();
		//}
		//#endif

		ret->push_back(PointT(cloest[0], cloest[1], cloest[2]));
	}
	return ret;
}

CP OMPSurface::reconstruction()
{
	CzxTimer _(__func__);
	CP reconstructed(new CloudT);
	#pragma omp parallel for
	for (int i = 0; i < sub_clouds.size(); i++)
	{
		if (sub_clouds[i].cloud == NULL)
			continue;
		CP region = reconstruction(*sub_clouds[i].fit, sub_clouds[i].cloud);
		#pragma omp critical
		{
			*reconstructed += *region;
		}
	}
	return reconstructed;
}

#ifndef NEW_NURBS
bool OMPSurface::fitting(ROICloud& cloud_in, int N)
{
	CzxTimer dsdg(__func__);

	double success_ret = 0.001;

	N = cloud_in.cloud->size() / 3400;
	if (N < 1) N = 1;

	CP cloud(new CloudT);
	cloud = arsenal::downSampleByIndex<CloudT>(cloud_in.cloud, N);

	int order(3);
	int refinement(5);
	int iteration(1);
	//int mesh_resolution(256);

	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;


	//m_data->interior.colwise().mean();
	//m_data->interior.colwise() -= mean;

	//Vector3f mean = cloud->getMatrixXdMap(3, 4, 0).colwise().mean();
	//cloud->getMatrixXdMap(3, 4, 0).colwise() -= mean;

	pcl::on_nurbs::NurbsDataSurface data;
	PointCloud2Vector3d(cloud, data.interior);
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCA(order, &data);

	//NurbsSurface ns;
	//NurbsDataSurface n_data;
	//n_data.interior = cloud->getMatrixXdMap(3, 4, 0);
	//ns.initNurbsPCA(order, &n_data);

	//NurbsFit nfit();

	//{
	//	CP ori = getNurb(nurbs);
	//	//Tool::showComparison(ori, cloud);
	//	CP ori_cvs(new CloudT);
	//	for(int i=0;i<3;i++)
	//		for (int j = 0; j < 3; j++)
	//		{
	//			ON_3dPoint p;
	//			nurbs.GetCV(i, j, p);
	//			ori_cvs->push_back(PointT(p.x, p.y, p.z));
	//		}
	//	CP my = getNurb(ns);
	//	CP my_cvs(new CloudT);
	//	for (int i = 0; i < 3; i++)
	//		for (int j = 0; j < 3; j++)
	//		{
	//			double* cv = ns.CV(i, j);
	//			my_cvs->push_back(PointT(cv[0], cv[1], cv[2]));
	//		}
	//	Tool::showComparison(my_cvs, ori_cvs);
	//	Tool::showComparison(my, cloud);
	//}

	#ifdef CZX_DEBUG
	CP last(new CloudT);
	#endif

	CP ret;
	cloud_in.fit = new pcl::on_nurbs::FittingSurface(&data, nurbs);
	//cloud_in.fit = new DerivedFittingSurface(&data, nurbs);

	//pcl::on_nurbs::NurbsSolve m_solver;
	//m_solver.assign(1, 12, 3);


	auto& fit_ptr = cloud_in.fit;

	for (int i = 0; i < refinement; i++)
	{
		{
			CzxTimer sgag(std::to_string(i));
			fit_ptr->refine(0);
			if(i%2==0)
				fit_ptr->refine(1);
			{
				CzxTimer sgafhafhaw("assemble" + std::to_string(i));
				fit_ptr->assemble(params);
			}
			fit_ptr->solve();
			//*ret += *cloud;
		}

		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//Tool::showComparison(cloud, ret);
		//#endif

		{
			cloud_in.profile = surfaceProfile(*fit_ptr, cloud);
			if (cloud_in.profile < success_ret)
				return true;
		}


		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//if (i >= 1)
		//{
		//	//comparisonTwoCloud(last, ret, 0.01);
		//	//comparisonTwoCloud(ret, cloud, 0.01);
		//	Tool::showComparison(last, ret, cloud);
		//}
		//*last = *ret;
		//#endif
	}

	for (int i = 0; i < iteration; i++)
	{
		{
			CzxTimer sgag(std::to_string(i));
			fit_ptr->assemble(params);
			fit_ptr->solve();
			{
				cloud_in.profile = surfaceProfile(*fit_ptr, cloud);
				if (cloud_in.profile < success_ret)
					return true;
			}
		}
		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//comparisonTwoCloud(last, ret, 0.01);
		//*last = *ret;
		//#endif
		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//Tool::showComparison(cloud, ret);
		//#endif
	}

	if (cloud_in.cloud->size() >= 40000)
	{
		auto subsub_clouds = devideCloud(cloud_in.cloud, 4);
		cloud_in.cloud = NULL;
		#pragma omp parallel for
		for (int i = 0; i < subsub_clouds.size(); i++)
		{
			auto flag = fitting(subsub_clouds[i], 1);	
		}
		mtx_subcloud.lock();
		sub_clouds.insert(sub_clouds.end(), subsub_clouds.begin(), subsub_clouds.end());
		mtx_subcloud.unlock();
		return false;
	}


	// curve fitting
	//pcl::PolygonMesh mesh;
	// parameters
	//pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	//{
	//	curve_params.addCPsAccuracy = 5e-2;
	//	curve_params.addCPsIteration = 3;
	//	curve_params.maxCPs = 200;
	//	curve_params.accuracy = 1e-3;
	//	curve_params.iterations = 100;

	//	curve_params.param.closest_point_resolution = 0;
	//	curve_params.param.closest_point_weight = 1.0;
	//	curve_params.param.closest_point_sigma2 = 0.1;
	//	curve_params.param.interior_sigma2 = 0.00001;
	//	curve_params.param.smooth_concavity = 1.0;
	//	curve_params.param.smoothness = 1.0;
	//}
	// initialisation (circular)
	//printf("  curve fitting ...\n");
	//pcl::on_nurbs::NurbsDataCurve2d curve_data;
	//curve_data.interior = data.interior_param;
	//curve_data.interior_weight_function.push_back(true);
	//ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

	//pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	//curve_fit.fitting(curve_params);

	 //############################################################################
	// triangulation of trimmed surface

	//printf("  triangulate trimmed surface ...\n");
	//pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
	//	mesh_resolution);

	//pcl::visualization::PCLVisualizer viewer;
	//viewer.addPolygonMesh(mesh);
	//viewer.addPointCloud(cloud, "2");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
	//viewer.spin();

	return false;
}
#endif

#ifdef NEW_NURBS

#define VERIFY_AND_RETURN_TRUE \
    if (Verify(*fit_ptr, cloud_roi.cloud, success_ret, cloud_roi.cloud->size()*0.05)) \
        return true;


bool OMPSurface::fitting_2(ROICloud& cloud_roi, int N)
{
	//CzxTimer agafh(__func__);
	int order(3);
	int refinement(5);
	int iteration(1);

	Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;

	NurbsSurface ns;
	NurbsDataSurface n_data;

	int min_cloud_size = stoi(conf["min_cloud_size"]);

	N = cloud_roi.cloud->size() / min_cloud_size;
	if (N < 1) N = 1;
	CP cloud_down(new CloudT);
	cloud_down = arsenal::downSampleByIndex<CloudT>(cloud_roi.cloud, N);
	n_data.interior = cloud_down->getMatrixXfMap(3, 4, 0).cast<double>();

	//n_data.interior = cloud_roi.cloud->getMatrixXfMap(3, 4, 0).cast<double>();

	ns.initNurbsPCA(order, &n_data);

	CP ret;
	cloud_roi.fit = new NurbsFit(&n_data, ns);

	auto& fit_ptr = cloud_roi.fit;

	double success_ret = 0.02;

	for (int i = 0; i < refinement; i++)
	{
		{
			//CzxTimer sgag(std::to_string(i));
			fit_ptr->refine(0);
			if (i % 2 == 0)
				fit_ptr->refine(1);
			{
				//CzxTimer sgafhafhaw("assemble" + std::to_string(i));
				fit_ptr->assemble(params);
			}
			{
				//CzxTimer sgafhafhaw("solve" + std::to_string(i));
				fit_ptr->solve();
			}
			//*ret += *cloud;
		}

		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//Tool::showComparison(cloud, ret);
		//#endif

		{
			//cloud_roi.profile = surfaceProfile(*fit_ptr, cloud_down);
			//if (cloud_roi.profile < success_ret)
				//return true;
			VERIFY_AND_RETURN_TRUE
			//if (Verify(*fit_ptr, cloud_roi.cloud, success_ret, 10))
			//	return true;
		}


		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//if (i >= 1)
		//{
		//	//comparisonTwoCloud(last, ret, 0.01);
		//	//comparisonTwoCloud(ret, cloud, 0.01);
		//	Tool::showComparison(last, ret, cloud);
		//}
		//*last = *ret;
		//#endif
	}

	for (int i = 0; i < iteration; i++)
	{
		{
			//CzxTimer sgag(std::to_string(i));
			fit_ptr->assemble(params);
			fit_ptr->solve();
			{
				//cloud_roi.profile = surfaceProfile(*fit_ptr, cloud_roi.cloud);
				//if (cloud_roi.profile < success_ret)
				//	return true;
				VERIFY_AND_RETURN_TRUE
				//if (Verify(*fit_ptr, cloud_roi.cloud, success_ret, 10))
				//	return true;
			}
		}
		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//comparisonTwoCloud(last, ret, 0.01);
		//*last = *ret;
		//#endif
		//#ifdef CZX_DEBUG
		//ret = getNurb(fit.m_nurbs);
		//Tool::showComparison(cloud, ret);
		//#endif
	}


	if (cloud_roi.cloud->size() >= min_cloud_size *4*4)
	{
		VRC subsub_clouds;
		devideCloud(cloud_roi.cloud, 4, subsub_clouds);
		for (int i = 0; i < subsub_clouds.size(); i++)
			if (subsub_clouds[i].cloud->size() < min_cloud_size)
				return false;
		cloud_roi.cloud = NULL;
		#pragma omp parallel for
		for (int i = 0; i < subsub_clouds.size(); i++)
		{
			auto flag = fitting_2(subsub_clouds[i], 1);	
		}
		mtx_subcloud.lock();
		sub_clouds.insert(sub_clouds.end(), subsub_clouds.begin(), subsub_clouds.end());
		mtx_subcloud.unlock();
		return false;
	}


	return false;
}
#endif

void OMPSurface::process()
{
	CzxTimer asdfgbhajsdbgj(__func__);

	//CP ret(new CloudT);
	int size_sub = sqrt(zone_num);
	max_profile = -999;

	NurbsSolve::init();

	//omp_set_num_threads(20);
	#pragma omp parallel for //num_threads(20)
	for (int ij = 0; ij < zone_num; ij++)
	{
		int i = ij / size_sub;
		int j = ij % size_sub;
		#ifdef NEW_NURBS
		auto flag = fitting_2(sub_clouds[ij], 10);
		#else
		auto flag = fitting(sub_clouds[ij], 10);
		#endif
	}
	#ifdef CZX_DEBUG
	CP reconstructed(new CloudT);
	CP sub_sum(new CloudT);
	#endif

	#pragma omp parallel for
	for (int i = 0; i < sub_clouds.size(); i++)
	{
		//*ret += *fitted_cloud;
		if (sub_clouds[i].cloud == NULL)
			continue;

		auto profile = surfaceProfile(*sub_clouds[i].fit, sub_clouds[i].cloud);

		#ifdef CZX_DEBUG
		CP region = reconstruction(*sub_clouds[i].fit, sub_clouds[i].cloud);
		#pragma omp critical
		{
			*reconstructed += *region;
			*sub_sum += *sub_clouds[i].cloud;
		}
		#endif

		#pragma omp critical
		if (max_profile < profile)
		{
			#ifdef CZX_DEBUG
			if (profile > 1)
			{
				cout << "new max profile:" << profile << endl;
				//arsenal::comparisonTwoCloud(main_cloud, sub_clouds[i].cloud,0.01);
				arsenal::comparisonTwoCloud(main_cloud, reconstruction(*sub_clouds[i].fit, sub_clouds[i].cloud), 0.01);
			}
			#endif
			max_profile = profile;
		}
	}

	#ifdef CZX_DEBUG
	cout << max_profile << endl;
	//Tool::showComparison(main_cloud, reconstructed);
	arsenal::comparisonTwoCloud(main_cloud, reconstructed, 0.02);
	//arsenal::comparisonTwoCloud(main_cloud, sub_sum, 0.001);
	#endif

	NurbsSolve::finalize();
	//return ret;
}