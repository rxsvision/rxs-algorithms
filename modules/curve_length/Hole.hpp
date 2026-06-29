#pragma once

#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include<string.h>
#include<pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include<pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include<omp.h>
#include<pcl/surface/convex_hull.h>
#include<pcl/surface/concave_hull.h>
#include"types_.h"
#include"czxRansac.h"

class preProcess
{
public:
	unordered_map<string, string> config;
	CP backup;
	bool state;
	CP bound;
	string cur_path;

public:

	preProcess(string conf="conf.czx") : backup(new CloudT)
	{
		state = true;
		config = czxTool::readProfile(conf);
	}

	CP maxBlock(CP cloud)
	{
		CP cloud_filtered(new CloudT);

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		std::vector<pcl::PointIndices> cluster_indices;
		ec.setClusterTolerance(stof(config["maxBlock_euc_thres"]));
		ec.setMinClusterSize(50);
		ec.setMaxClusterSize(600000);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);


		//cout << cluster_indices.size() << endl;

		pcl::ExtractIndices<pcl::PointXYZ> extract;

		//vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;


		//for (auto index : cluster_indices)
		//{
		//	extract.setInputCloud(cloud);
		//	extract.setIndices(boost::make_shared<pcl::PointIndices>(index));
		//	extract.filter(*cloud_filtered);
		//	Tool::showComparison(cloud, cloud_filtered);
		//}

		extract.setInputCloud(cloud);
		extract.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices[0]));
		extract.filter(*cloud_filtered);
		//Tool::showComparison(cloud, cloud_filtered);


		return cloud_filtered;
	}

	Eigen::Matrix3f
		modelNormalize(CP cloud, CP cloud_projected)
	{
		pcl::PCA<PointT> pca;
		pca.setInputCloud(cloud);
		Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
		if (projection(2, 2) < 0)
		{
			projection = -projection;
		}

		//czxTool::printMatrixXf(projection);

		//pcl::transformPointCloud(*cloud, *cloud_projected, projection);

		Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_eigen = cloud->getMatrixXfMap(3, 4, 0);

		cloud_eigen = projection * cloud_eigen;

		cloud_projected->resize(cloud->size());
		cloud_projected->getMatrixXfMap(3, 4, 0) = cloud_eigen;

		// 恢复投影数据
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reconstructed(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::transformPointCloud(*cloud_projected, *cloud_reconstructed, projection.inverse());

		//Tool::showComparison(cloud, cloud_projected, true);
		return projection;
	}


	bool ellipseValid(CP cloud_in)
	{
		CP cloud(new CloudT);
		modelNormalize(cloud_in, cloud);
		pcl::PointXYZ min_point, max_point;
		pcl::getMinMax3D(*cloud, min_point, max_point);
		float dx = max_point.x - min_point.x;
		float dy = max_point.y - min_point.y;

		if (dx < 1.0 || dy < 1.0 || dx>10 || dy>10) return false;

		return true;
	}


	CP generatePlan()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// 定义平面的大小
		double plane_size = 2.0; // 平面边长为2

		// 计算平面上点的坐标并添加到点云中
		for (double x = -plane_size / 2; x <= plane_size / 2; x += 0.01) {
			for (double y = -plane_size / 2; y <= plane_size / 2; y += 0.01) {
				pcl::PointXYZ point;
				point.x = x;
				point.y = y;
				point.z = 0.0; // 平面位于XY平面上
				cloud->push_back(point);
			}
		}
		return cloud;
	}

	void removeRectangle(CP cloud)
	{
		//割一圈把长方形切掉
		{
			pcl::PointXYZ min_point, max_point;
			pcl::getMinMax3D(*cloud, min_point, max_point);
			float dx = max_point.x - min_point.x;
			float dy = max_point.y - min_point.y;

			pcl::PassThrough<pcl::PointXYZ> pass;
			//CP copy(new CloudT(*cloud));
			while (dy > 10 || dx > 10)
			{
				if (dy > 10)
				{
					pass.setInputCloud(cloud);
					pass.setFilterFieldName("x");
					pass.setFilterLimits(min_point.x + 1, max_point.x - 1);
					pass.filter(*cloud);
					pcl::getMinMax3D(*cloud, min_point, max_point);
					dy = max_point.y - min_point.y;
					dx = max_point.x - min_point.x;
				}
				if (dx > 10)
				{
					pass.setInputCloud(cloud);
					pass.setFilterFieldName("y");
					pass.setFilterLimits(min_point.y + 1, max_point.y - 1);
					pass.filter(*cloud);
					pcl::getMinMax3D(*cloud, min_point, max_point);
					dx = max_point.x - min_point.x;
					dy = max_point.y - min_point.y;
				}
				//Tool::showComparison(copy, cloud);
			}
		}
	}

	PointT findHole(CP cloud_in)
	{
		CP cloud_copy(new CloudT);
		CP cloud(new CloudT);
		pcl::copyPointCloud(*cloud_in, *cloud);

		// 创建均匀下采样滤波器
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.1f, 0.1f, 0.1f); // 设置下采样的体素大小
		sor.filter(*cloud); // 执行下采样
		//Tool::showComparison(cloud_in, cloud);
		// 在副本中遍历每个点，将 Z 坐标设置为零
		for (size_t i = 0; i < cloud_copy->size(); ++i) {
			cloud_copy->points[i].z = 0.0;
		}
		pcl::copyPointCloud(*cloud_in, *cloud_copy);

		//Tool::show(cloud_copy);

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(cloud);
		outrem.setRadiusSearch(std::stof(config["range_bound"]));
		outrem.setMinNeighborsInRadius(stoi(config["min_pts_bound"]));

		outrem.setNegative(true);
		pcl::shared_ptr<std::vector<int>> inliers(new std::vector<int>);
		outrem.filter(*inliers);
		{
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			//extract.setInputCloud(cloud_copy);
			extract.setIndices(inliers);
			extract.setNegative(false);
			//extract.filter(*cloud_copy);
			extract.setInputCloud(cloud);
			//CP copy(new CloudT(*cloud));
			extract.filter(*cloud);
			//Tool::showComparison(copy, cloud);
		}

		removeRectangle(cloud);

		////割一圈把长方形切掉
		//{
		//	pcl::PointXYZ min_point, max_point;
		//	pcl::getMinMax3D(*cloud, min_point, max_point);
		//	float dx = max_point.x - min_point.x;
		//	float dy = max_point.y - min_point.y;

		//	pcl::PassThrough<pcl::PointXYZ> pass;

		//	while (dy > 10 || dx > 10 )
		//	{
		//		if (dy > 10)
		//		{
		//			pass.setInputCloud(cloud);
		//			pass.setFilterFieldName("x");
		//			pass.setFilterLimits(min_point.x + 1, max_point.x - 1);
		//			pass.filter(*cloud);
		//			pcl::getMinMax3D(*cloud, min_point, max_point);
		//			dy = max_point.y - min_point.y;
		//			dx = max_point.x - min_point.x;
		//		}
		//		if (dx > 10)
		//		{
		//			pass.setInputCloud(cloud);
		//			pass.setFilterFieldName("y");
		//			pass.setFilterLimits(min_point.y + 1, max_point.y - 1);
		//			pass.filter(*cloud);
		//			pcl::getMinMax3D(*cloud, min_point, max_point);
		//			dx = max_point.x - min_point.x;
		//			dy = max_point.y - min_point.y;

		//		}
		//	}
		//}
		// 
		//Tool::show(cloud);

		Eigen::VectorXf rowMean = cloud->getMatrixXfMap(3, 4, 0).rowwise().mean();

		return PointT(rowMean[0], rowMean[1], rowMean[2]);
	}


	PointT centerFit(CP cloud_in) {

		CP cloud(new CloudT);
		pcl::copyPointCloud(*cloud_in, *cloud);

		Ransac ran;


		Eigen::Matrix3f norm2ellipseNorm = modelNormalize(cloud, cloud);

		Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_eigen = cloud->getMatrixXfMap(3, 4, 0);

		Eigen::VectorXf rowMean = cloud_eigen.rowwise().mean();

		cloud_eigen -= rowMean.replicate(1, cloud_eigen.cols());

		cloud_eigen /= 5;

		cloud->getMatrixXfMap(3, 4, 0) = cloud_eigen;

		//Tool::showComparison(cloud, generatePlan(), 5, 2);

		//pcl::io::savePCDFile("center", *cloud);
		bool success = ran.fit(cloud);
		//CP fitted_ellipse = ran.drawEllipse();
		//Tool::showComparison(cloud, fitted_ellipse, 5, 5, callback);
		if (!success)
		{
			state = false;
			return PointT();
		}

		PointT p(ran.getx(), ran.gety(), cloud_eigen.rowwise().mean()[2]);

		Eigen::Vector3f eigen_p(ran.getx(), ran.gety(), cloud_eigen.rowwise().mean()[2]);
		eigen_p *= 5;
		eigen_p += rowMean;

		eigen_p = norm2ellipseNorm.inverse() * eigen_p;

		p.x = eigen_p[0];
		p.y = eigen_p[1];
		p.z = eigen_p[2];

		//CP tmp(new CloudT);
		//tmp->push_back(p);


		return p;
	}

	CP process(CloudT::Ptr cloud)
	{
		CP keypoint(new CloudT);
		bound.reset(new CloudT);

		CP LD(new CloudT);
		{
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("x");
			//pass.setFilterLimits(-120, -100);
			pass.setFilterLimits(-100, -80);

			pass.filter(*LD);
			pass.setInputCloud(LD);
			pass.setFilterFieldName("y");
			//pass.setFilterLimits(300, 330);
			pass.setFilterLimits(120, 140);

			pass.filter(*LD);
		}


		CP RD(new CloudT);
		{
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("x");
			//pass.setFilterLimits(-120, -90);
			pass.setFilterLimits(-100, -80);

			pass.filter(*RD);
			pass.setInputCloud(RD);
			pass.setFilterFieldName("y");
			//pass.setFilterLimits(20, 50);
			pass.setFilterLimits(-160, -140);

			pass.filter(*RD);
		}

		CP LU(new CloudT);
		{
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("x");
			//pass.setFilterLimits(70, 90);
			pass.setFilterLimits(70, 90);
			pass.filter(*LU);
			pass.setInputCloud(LU);
			pass.setFilterFieldName("y");
			//pass.setFilterLimits(310, 330);
			pass.setFilterLimits(120, 140);
			pass.filter(*LU);
		}

		//*cloud = CloudT();

		PointT LD_key, RD_key, LU_key;
		#pragma omp parallel
		{
			#pragma omp sections
			{
				#pragma omp section
				{
					LD_key = gatherKeyPoint(LD);
					//*bound += *LD;
				}
				#pragma omp section
				{
					RD_key = gatherKeyPoint(RD);
					//*bound += *RD;
				}
				#pragma omp section
				{
					LU_key = gatherKeyPoint(LU);
					//*bound += *LU;
				}
			}
		}

		keypoint->push_back(LD_key);
		*bound += *LD;
		keypoint->push_back(RD_key);
		*bound += *RD;
		keypoint->push_back(LU_key);
		*bound += *LU;
		//if (!state) return CP();
		return keypoint;
	}

	CP denseFilter23D(CP cloud)
	{
		CP copy(new CloudT(*cloud));
		copy->getMatrixXfMap(3, 4, 0).row(2).setZero();
		//pcl::io::savePCDFile("cloud.pcd", *copy);

		pcl::IndicesPtr indices(new pcl::Indices);
		pcl::RadiusOutlierRemoval<PointT> ror;
		ror.setInputCloud(copy);
		ror.setRadiusSearch(stof(config["denseFilter_range"]));
		ror.setMinNeighborsInRadius(stoi(config["denseFilter_min_pts"]));
		ror.setNegative(true);
		ror.filter(*indices);

		CP filtered(new CloudT);
		pcl::ExtractIndices<PointT> ext;
		ext.setInputCloud(cloud);
		ext.setIndices(indices);
		ext.filter(*filtered);
		return filtered;
	}

	//CP xyZFilter(CP cloud)
	//{
	//	CP copy(new CloudT(*cloud));
	//	//copy->getMatrixXfMap(3, 4, 0).row(2) *= 10;
	//	pcl::io::savePCDFile("cloud.pcd", *copy);

	//	pcl::IndicesPtr indices(new pcl::Indices);
	//	pcl::RadiusOutlierRemoval<PointT> ror;
	//	ror.setInputCloud(copy);
	//	ror.setRadiusSearch(stof(config["xyZFilter_range"]));
	//	ror.setMinNeighborsInRadius(stoi(config["xyZFilter_min_pts"]));
	//	//ror.setNegative(true);
	//	ror.filter(*indices);

	//	CP filtered(new CloudT);
	//	pcl::ExtractIndices<PointT> ext;
	//	ext.setInputCloud(cloud);
	//	ext.setIndices(indices);
	//	ext.filter(*filtered);
	//	return filtered;
	//}

	/*
	PointT gatherKeyPoint(CP cloud)
	{
		Eigen::Matrix3f origin2norm = modelNormalize(cloud, cloud);
		CP copy(new CloudT);

		CP cloud_filtered = filterNormal(cloud);
		//CP cloud_filtered = xyZFilter(cloud);
		Tool::showComparison(cloud, cloud_filtered);
		return PointT();

		pcl::copyPointCloud(*cloud_filtered, *copy);
		cloud_filtered = maxBlock(cloud_filtered);
		Tool::showComparison(copy, cloud_filtered);
		return PointT();

		pcl::copyPointCloud(*cloud_filtered, *copy);
		//cloud_filtered = BoundExtract(cloud_filtered);
		cloud_filtered = KNNBoundExtract(cloud_filtered);
		//Tool::show(cloud_filtered);
		//Tool::showComparison(copy, cloud_filtered);
		//return PointT();

		if (!state) return PointT();

		PointT p = centerFit(cloud_filtered);
		//CP tmp(new CloudT);
		//tmp->push_back(p);
		//Tool::showComparison(cloud_filtered, tmp, 5, 15);
		p.getVector3fMap() = origin2norm.transpose() * p.getVector3fMap();

		return p;

		//return PointT();
	}
	*/

	PointT gatherKeyPoint(CP cloud)
	{
		Eigen::Matrix3f origin2norm = modelNormalize(cloud, cloud);
		CP copy(new CloudT);
		CP cloud_filtered;

		//Tool::show(cloud);
		cloud_filtered = KNNBoundExtract(cloud)[0];
		//Tool::showComparison(cloud, cloud_filtered, 3, 2, callback);


		PointT p = centerFit(cloud_filtered);
		//CP tmp(new CloudT);
		//tmp->push_back(p);
		//Tool::showComparison(cloud_filtered, tmp, 5, 15);
		p.getVector3fMap() = origin2norm.transpose() * p.getVector3fMap();
		cloud_filtered->getMatrixXfMap(3, 4, 0) = origin2norm.transpose() * cloud_filtered->getMatrixXfMap(3, 4, 0);
		*cloud = *cloud_filtered;
		return p;

		/*
		cloud_filtered = filterNormal(cloud);
		//CP cloud_filtered = xyZFilter(cloud);
		Tool::showComparison(cloud, cloud_filtered);
		return PointT();

		pcl::copyPointCloud(*cloud_filtered, *copy);
		cloud_filtered = maxBlock(cloud_filtered);
		Tool::showComparison(copy, cloud_filtered);
		return PointT();

		pcl::copyPointCloud(*cloud_filtered, *copy);
		//cloud_filtered = BoundExtract(cloud_filtered);
		//Tool::show(cloud_filtered);
		//Tool::showComparison(copy, cloud_filtered);
		//return PointT();

		if (!state) return PointT();

		//PointT p = centerFit(cloud_filtered);
		//CP tmp(new CloudT);
		//tmp->push_back(p);
		//Tool::showComparison(cloud_filtered, tmp, 5, 15);
		//p.getVector3fMap() = origin2norm.transpose() * p.getVector3fMap();

		//return p;

		//return PointT();
		*/
	}

	PointT gatherCleanPoint(CP cloud)
	{
		Eigen::Matrix3f origin2norm = modelNormalize(cloud, cloud);
		CP copy(new CloudT(*cloud));
		//cloud = filterNormal(cloud);
		//Tool::showComparison(copy, cloud);

		//cloud = maxBlock(cloud);
		//Tool::showComparison(copy, cloud);
		//return PointT();
		cloud = KNNBoundExtract(cloud)[0];
		//Tool::showComparison(copy, cloud, 5, 5);
		return PointT();
		PointT p = centerFit(cloud);
		//CP tmp(new CloudT);
		//tmp->push_back(p);
		//Tool::showComparison(cloud_filtered, tmp, 5, 15);
		p.getVector3fMap() = origin2norm.transpose() * p.getVector3fMap();

		return p;
	}

	static bool compareAngle(PointT a, PointT b, PointT c)
	{
		return (((a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)) >= 0);
	}

	float xyDis(PointT a, PointT b)
	{
		return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
	}

	vector<CP> Hull(CP cloud, PointT p_useless)
	{
		Eigen::VectorXf rowMean = cloud->getMatrixXfMap(3, 4, 0).rowwise().mean();

		PointT p(rowMean[0], rowMean[1], rowMean[2]);

		auto compare = [&p](PointT a, PointT b) {
			return compareAngle(a, b, p);
		};
		sort(cloud->begin(), cloud->end(), compare);

		//for (auto p : *cloud)
		//{
		//	tmp->push_back(p);
		//	Tool::showComparison(cloud, tmp, 5, 7);
		//}
		//TODO
		//这里需要评价重筛选避免第一次选中噪点
		int start = 0;
		PointT last = cloud->points[start];

		vector<PointT> starts;
		vector<CP> clouds;
		vector<PointT> lasts;
		//vector<float> mins;
		vector<float> maxs;


		starts.push_back(cloud->points[0]);
		lasts.push_back(cloud->points[0]);
		maxs.push_back(xyDis(cloud->points[0], p));

		{
			CP tmp(new CloudT);
			tmp->push_back(cloud->points[0]);
			clouds.push_back(tmp);
		}

		//{
		//	CP tmp(new CloudT);
		//	for (int i = 0; i < cloud->size(); i++)
		//	{
		//		tmp->push_back(cloud->points[i]);
		//		clouds.push_back(tmp);
		//		Tool::showComparison(cloud, tmp, 3, 6);
		//	}
		//}

		for (int i = 1; i < cloud->size(); i++)
		{
			PointT cur_p = cloud->points[i];
			bool found = false;
			for (int j = 0; j < lasts.size(); j++)
			{
				if (abs(xyDis(lasts[j], p) - xyDis(cur_p, p)) < stof(config["convexHull_threshold_xy"]) &&
					abs(lasts[j].z - cur_p.z) < stof(config["convexHull_threshold_z"]) &&
					(abs(starts[j].z - cur_p.z) < stof(config["convexHull_threshold_z_total"])
						//	|| (xyDis(lasts[j], p)>2 && abs(lasts[j].z - cur_p.z) < (stof(config["convexHull_threshold_z"]) * 1.5))
						)
					)
				{
					clouds[j]->push_back(cur_p);
					lasts[j] = cur_p;
					float bias = xyDis(cur_p, p);
					maxs[j] = max(maxs[j], bias);
					found = true;
					break;
				}

			}
			if (!found)
			{
				CP tmp(new CloudT);
				tmp->push_back(cur_p);

				//if (clouds.size() == 4)
				//{
				//	Tool::showComparison(tmp, clouds[1]);
				//	CP last_point(new CloudT);
				//	last_point->push_back(lasts[1]);
				//	pcl::io::savePCDFileBinary("cloud.pcd", *last_point);
				//	pcl::io::savePCDFileBinary("tmp.pcd", *tmp);
				//}
				//for (int x = 0; x < clouds.size(); x++)
				//{
				//	Tool::showComparison(clouds[x], tmp, 3, 5);
				//}

				clouds.push_back(tmp);
				starts.push_back(cur_p);
				lasts.push_back(cur_p);
				maxs.push_back(xyDis(cur_p, p));

			}
		}

		vector<float> dists_start;
		vector<float> dists_end;
		vector<float> difs;

		float max_bound = 0;


		vector<CP> ret;
		for (auto cl : clouds)
		{
			if (cl->size() < 200)continue;
			ret.push_back(cl);
		}

		std::sort(ret.begin(), ret.end(), [&](CP a, CP b) {
			//return a->size() > b->size();
			return a->getMatrixXfMap(3, 4, 0).row(2).mean() > b->getMatrixXfMap(3, 4, 0).row(2).mean();
			});

		cloud->push_back(p);


		return ret;
	}

	vector<CP> KNNBoundExtract(CP cloud, bool hull = true)
	{
		CP filtered(new CloudT);
		// 创建一个KdTree对象
		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(cloud);

		// 定义K值，即寻找的最近邻的数量
		int K;// = stoi(config["K"]);

		// 迭代遍历每个点并查找其K近邻
		for (size_t i = 0; i < cloud->size(); ++i) {
			PointT searchPoint = cloud->points[i]; // 当前点
			std::vector<int> pointIdxNKNSearch; // 用于存储最近邻点的索引
			std::vector<float> pointNKNSquaredDistance; // 用于存储最近邻点的距离的平方

			float radius = stof(config["radius_bound"]);
			if (kdtree.radiusSearch(searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
				K = pointIdxNKNSearch.size();
				if (K > stof(config["knnNeighbor"])) continue;

				float sumX = 0.0;
				float sumY = 0.0;
				float sumZ = 0.0;
				float maxX = -999999999;
				float maxY = -999999999;
				float maxZ = -999999999;
				float minX = std::numeric_limits<float>::max();
				float minY = std::numeric_limits<float>::max();
				float minZ = std::numeric_limits<float>::max();
				for (int j = 0; j < K; ++j) {
					int neighborIdx = pointIdxNKNSearch[j];
					sumX += cloud->points[neighborIdx].x;
					sumY += cloud->points[neighborIdx].y;
					sumZ += cloud->points[neighborIdx].z;
					maxX = max(maxX, cloud->points[neighborIdx].x);
					maxY = max(maxY, cloud->points[neighborIdx].y);
					maxZ = max(maxZ, cloud->points[neighborIdx].z);

					minX = min(minX, cloud->points[neighborIdx].x);
					minY = min(minY, cloud->points[neighborIdx].y);
					minZ = min(minZ, cloud->points[neighborIdx].z);
				}
				float meanX = abs(sumX / K - searchPoint.x);
				float meanY = abs(sumY / K - searchPoint.y);
				float meanZ = abs(sumZ / K - searchPoint.z);


				float dif = max(meanX, meanY);

				if (dif > stof(config["knnXYZ"]))
				{
					filtered->push_back(searchPoint);
				}
			}
		}



		PointT center = findHole(cloud);
		{
			CP copy(new CloudT(*filtered));
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(filtered);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(center.x - 3, center.x + 3);
			pass.filter(*filtered);
			pass.setInputCloud(filtered);
			pass.setFilterFieldName("y");
			pass.setFilterLimits(center.y - 3, center.y + 3);
			pass.filter(*filtered);
			//Tool::showComparison(cloud, filtered);
		}
		vector<CP> ret;
		if (hull)
			ret = Hull(filtered, center);
		else
		{
			removeRectangle(filtered);
			ret.push_back(filtered);
		}
		return ret;
	}
};