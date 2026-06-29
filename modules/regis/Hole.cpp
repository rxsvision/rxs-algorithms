#include "Hole.h"


CP Hole::KNNBoundExtract(CP cloud)
{
	CzxTimer gadg(__func__);
	CP filtered(new CloudT);
	// 创建一个KdTree对象
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// 定义K值，即寻找的最近邻的数量
	// = stoi(config["K"]);

	// 迭代遍历每个点并查找其K近邻
	//#pragma omp parallel for num_threads(3)
	for (int i = 0; i < cloud->size(); ++i) 
	{
		PointT searchPoint = cloud->points[i]; // 当前点
		std::vector<int> pointIdxNKNSearch; // 用于存储最近邻点的索引
		std::vector<float> pointNKNSquaredDistance; // 用于存储最近邻点的距离的平方

		float radius = stof(config["radius_bound"]);
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			int K;
			K = pointIdxNKNSearch.size();
			if (K > stof(config["knnNeighbor"])) continue;

			float sumX = 0.0;
			float sumY = 0.0;
			//float sumZ = 0.0;
			float maxX = -999999999;
			float maxY = -999999999;
			//float maxZ = -999999999;
			float minX = std::numeric_limits<float>::max();
			float minY = std::numeric_limits<float>::max();
			//float minZ = std::numeric_limits<float>::max();
			for (int j = 0; j < K; ++j) {
				int neighborIdx = pointIdxNKNSearch[j];
				sumX += cloud->points[neighborIdx].x;
				sumY += cloud->points[neighborIdx].y;
				//sumZ += cloud->points[neighborIdx].z;
				maxX = max(maxX, cloud->points[neighborIdx].x);
				maxY = max(maxY, cloud->points[neighborIdx].y);
				//maxZ = max(maxZ, cloud->points[neighborIdx].z);

				minX = min(minX, cloud->points[neighborIdx].x);
				minY = min(minY, cloud->points[neighborIdx].y);
				//minZ = min(minZ, cloud->points[neighborIdx].z);
			}
			float meanX = abs(sumX / K - searchPoint.x);
			float meanY = abs(sumY / K - searchPoint.y);
			//float meanZ = abs(sumZ / K - searchPoint.z);


			float dif = max(meanX, meanY);

			#pragma omp critical
			{
				if (dif > stof(config["knnXYZ"]))
				{
					filtered->push_back(searchPoint);
				}
			}
		}
	}

	return filtered;
};


PointT Hole::centerFit(CP cloud)
{
	//CzxTimer gadg(__func__);
	Ransac ran;

	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_eigen = cloud->getMatrixXfMap(3, 4, 0);

	Eigen::VectorXf rowMean = cloud_eigen.rowwise().mean();

	cloud_eigen -= rowMean.replicate(1, cloud_eigen.cols());

	cloud_eigen /= 5;

	cloud->getMatrixXfMap(3, 4, 0) = cloud_eigen;

	bool success = ran.fit(cloud);
	//CP fitted_ellipse = ran.drawEllipse();
	//Tool::showComparison(cloud, fitted_ellipse, 5, 5);
	if (!success)
	{
		return PointT();
	}

	PointT p(ran.getx(), ran.gety(), cloud_eigen.rowwise().mean()[2]);

	Eigen::Vector3f eigen_p(ran.getx(), ran.gety(), cloud_eigen.rowwise().mean()[2]);
	eigen_p *= 5;
	eigen_p += rowMean;

	p.x = eigen_p[0];
	p.y = eigen_p[1];
	p.z = eigen_p[2];

	//CP tmp(new CloudT);
	//tmp->push_back(p);
	//Tool::showComparison(cloud, tmp);
	return p;
}
