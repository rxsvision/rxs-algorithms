#pragma once
#include<pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointCT> CloudCT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> CloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef CloudT::Ptr CP;
