#include"czxTool.h"
#include<pcl/surface/on_nurbs/fitting_surface_pdm.h>
#include<pcl/common/pca.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include<pcl/filters/passthrough.h>
#include<pcl/common/distances.h>
#include"OMPSurface.h"
#include"nanoflann.hpp"

#define CZX_DEBUG

using namespace nanoflann;

struct PointCloudAdapter {
	const CP cloud;

	inline size_t kdtree_get_point_count() const { return cloud->points.size(); }

	inline float kdtree_distance(const float* p1, const size_t idx_p2, size_t /*size*/) const {
		const float d0 = p1[0] - cloud->points[idx_p2].x;
		const float d1 = p1[1] - cloud->points[idx_p2].y;
		return d0 * d0 + d1 * d1;
		//const float d2 = p1[2] - cloud.points[idx_p2].z;
		//return d0 * d0 + d1 * d1 + d2 * d2;  // Euclidean distance
	}

	inline float kdtree_get_pt(const size_t idx, int dim) const {
		switch (dim) {
		case 0: return cloud->points[idx].x;
		case 1: return cloud->points[idx].y;
		//case 2: return cloud.points[idx].z;
		}
		return 0;
	}

	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

CP surfaceFilter(CP cloud, unordered_map<string, string> config)
{
	CzxTimer _(__func__);
	PointCloudAdapter cloudAdapter{cloud };
	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloudAdapter>, PointCloudAdapter, 2> KDTree;
	KDTree index(2, cloudAdapter, KDTreeSingleIndexAdaptorParams(stoi(config["nano_leaf"])));
	const float search_radius = stof(config["radius_1"]) * stof(config["radius_1"]);
	int threshold = stoi(config["threshold_1"]);

	CP ret(new CloudT);

	SearchParameters params;
	params.sorted = false;

	{
		int thread_num = 20;
		vector<CP> omp_ret(thread_num);
		for (auto &value : omp_ret)
		{
			value.reset(new CloudT);
		}
		//CzxTimer _("search");
		#pragma omp parallel for num_threads(thread_num)
		for (int i = 0; i < cloud->size(); ++i)
		{
			vector<ResultItem<uint32_t, float>> ret_matches;
			float query_point[2] = { cloud->points[i].x, cloud->points[i].y };			
			auto num_matches = index.radiusSearch(query_point, search_radius, ret_matches, params);
			if (num_matches > threshold)
			{					
				//#pragma omp critical		
				//ret->push_back(cloud->points[i]);
				omp_ret[omp_get_thread_num()]->push_back(cloud->points[i]);
			}
		}
		for (auto value : omp_ret)
		{
			//ret->insert(ret->end(), value->begin(), value->end());
			*ret += *value;			
		}
	}
	return ret;
	//return cloud;

	//CzxTimer _(__func__);
	//pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>);
	//cloud_2d->resize(cloud->size());
	//for (size_t i = 0; i < cloud->points.size(); ++i) {
	//	cloud_2d->points[i].x = cloud->points[i].x;
	//	cloud_2d->points[i].y = cloud->points[i].y;
	//}

	//// 创建 KdTree2D对象
	//pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	//{
	//	CzxTimer _("建树");
	//	kdtree.setInputCloud(cloud_2d);
	//}
	//float radius = stof(config["radius_1"]);
	//int threshold = stoi(config["threshold_1"]);

	//CP ret(new CloudT);

	//#pragma omp parallel for
	//for (int i = 0; i < cloud->size(); ++i)
	//{
	//	pcl::PointXY searchPoint = cloud_2d->points[i]; // 当前点
	//	std::vector<int> pointIdxNKNSearch; // 用于存储最近邻点的索引
	//	std::vector<float> pointNKNSquaredDistance; // 用于存储最近邻点的距离的平方
	//	if (kdtree.radiusSearch(searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > threshold)
	//	{
	//		#pragma omp critical
	//		ret->push_back(cloud->points[i]);
	//	}
	//	//else if(kdtree.radiusSearch(searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 20)
	//	//{

	//	//}
	//}

	//return ret;
}

void main()
{
	//pcl::io::loadPCDFile("sb.pcd", *cloud);
	auto path_list = arsenal::pathGather("弧面/", "*.pcd");



	auto conf = czxTool::readProfile("conf_surface.czx");
	string path = "D:\\code\\surface\\20240717172059-911_0_X.pcd";
	CP cloud(new CloudT);
	pcl::io::loadPCDFile(path, *cloud);

	//HMODULE hDLL = LoadLibrary(L"czxToolkit.dll");
	//typedef double(__cdecl* DCI)(CP cloud, int zone_num);
	//DCI surfaceProfile = (DCI)GetProcAddress(hDLL, "surfaceProfile");
		//auto s = surfaceProfile(cloud, 12);

	cloud = surfaceFilter(cloud, conf);
	OMPSurface sur(12);
	sur.setInputCloud(cloud);
	sur.process();
	#ifdef RXS_HAS_VISUALIZATION
	Tool::showComparison(cloud, sur.reconstruction());
	#endif


	for (auto file_path : path_list)	
	{
		CP cloud(new CloudT);
		pcl::io::loadPCDFile(file_path, *cloud);

		//cout << "轮廓度: " << surfaceProfile(cloud, 12);
		//continue;

		{
			//surfaceFilter(cloud, conf);
			//Tool::showComparison(cloud, surfaceFilter(cloud, conf));
			//CzxComparison _(cloud);
			cloud = surfaceFilter(cloud, conf);
			//continue;
		}

		pcl::PCA<PointT> pca;
		pca.setInputCloud(cloud);
		Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
		cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);

		for (int _ = 0; _ < 1; _++)
		{
			CzxTimer sgsahf("surface");
			OMPSurface sur(6);
			sur.setInputCloud(cloud);
			sur.process();
			cout << sur.max_profile << endl;
		}
	}
}