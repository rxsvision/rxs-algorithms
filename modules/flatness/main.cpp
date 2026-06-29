#include"../czxDependence/czxTool.h"
#include"../czxDependence/czxTool_std.h"
#include<pcl/io/ply_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include"../czxDependence/nanoflann.hpp"
#include <pcl/segmentation/sac_segmentation.h>

using namespace nanoflann;

auto conf = czxTool::readProfile("flatness_conf.czx");
std::ofstream file("log/" + czx_file::getFileName(conf["root"]) + ".csv", std::ios::trunc);

VectorXf planeFitLSM(CP cloud)
{
    MatrixXf xyz = cloud->getMatrixXfMap(3, 4, 0);
    VectorXf z = xyz.row(2);
    xyz.row(2).setOnes();
    Eigen::VectorXf coff = Eigen::JacobiSVD<Eigen::MatrixXf>(xyz.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z);
    return coff;
}

/// <summary>
/// 计算平面度
/// </summary>
/// <param name="cloud">输入平面点云</param>
/// <param name="min_point">输出的最低点</param>
/// <param name="max_point">输出最高点</param>
/// <returns></returns>
float Flatness(CP cloud, PointT& min_point, PointT& max_point)
{
    CzxTimer dsghfa(__func__);
    auto coff = planeFitLSM(cloud);
    Vector3f normal = Vector3f(coff[0], coff[1], -1).normalized();
    VectorXf z = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    int max_index = 0;
    float max_value = z.maxCoeff(&max_index);
    int min_index = 0;
    float min_value = z.minCoeff(&min_index);
    min_point = cloud->points[min_index];
    max_point = cloud->points[max_index];
    return max_value - min_value;
}

struct PointCloudAdapter
{
    const CP cloud;

    inline size_t kdtree_get_point_count() const { return cloud->points.size(); }

    inline float kdtree_distance(const float* p1, const size_t idx_p2, size_t /*size*/) const
    {
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

CP KNNBoundRemove(CP cloud, float radius, int min_K)
{
    CzxTimer asdgsdgijsagi(__func__);
    CP filtered(new CloudT);
    // 创建一个KdTree对象
    PointCloudAdapter cloudAdapter{ cloud };
    typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloudAdapter>, PointCloudAdapter, 2> KDTree;
    KDTree index(2, cloudAdapter);
    SearchParameters params;
    params.sorted = false;

    // 迭代遍历每个点并查找其K近邻
    for (size_t i = 0; i < cloud->size(); ++i) {
        vector<ResultItem<uint32_t, float>> ret_matches;
        float query_point[2] = { cloud->points[i].x, cloud->points[i].y };
        auto num_matches = index.radiusSearch(query_point, radius* radius, ret_matches, params);

        if (num_matches > min_K) 
        {
            filtered->push_back(cloud->points[i]);
        }
    }
    return filtered;
}

CP KNNBound(CP cloud, float radius, int min_K)
{
    CzxTimer asdgsdgijsagi(__func__);
    CP filtered(new CloudT);
    // 创建一个KdTree对象
    PointCloudAdapter cloudAdapter{ cloud };
    typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloudAdapter>, PointCloudAdapter, 2> KDTree;
    KDTree index(2, cloudAdapter);
    SearchParameters params;
    params.sorted = false;

    // 迭代遍历每个点并查找其K近邻
    for (size_t i = 0; i < cloud->size(); ++i) {
        vector<ResultItem<uint32_t, float>> ret_matches;
        float query_point[2] = { cloud->points[i].x, cloud->points[i].y };
        auto num_matches = index.radiusSearch(query_point, radius * radius, ret_matches, params);

        if (num_matches < min_K)
        {
            filtered->push_back(cloud->points[i]);
        }
    }
    return filtered;
}


CP maxCluster(CP cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    // 欧几里得聚类提取器
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // 设置聚类的距离容忍度
    //ec.setMinClusterSize(10000); // 设置一个连通域的最小点数
    //ec.setMaxClusterSize(500000); // 设置一个连通域的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    {
        CzxTimer asdgsdgijsagi("extract");
        ec.extract(cluster_indices);
    }
    CP max_cluster(new CloudT);
    // 遍历所有的连通域
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
            cloud_cluster->points.push_back(cloud->points[idx]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if (cloud_cluster->size() > max_cluster->size())
            max_cluster = cloud_cluster;

        //cout << cloud_cluster->size() << endl;
        //Tool::showComparison(cloud, cloud_cluster);
    }
    return max_cluster;
}

CP planeExtractCircle(CP cloud)
{
    CzxTimer _(__func__);
    arsenal::passThrough(cloud, "z", stof(conf["z"]) - 2, stof(conf["z"]) + 2);
    auto plane_big = maxCluster(cloud);
    int radius_ = 5;
    float resolution = stof(conf["resolution"]);
    auto bound = KNNBound(plane_big, radius_ * resolution, 0.5 * radius_ * radius_ * 3.14);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // 调整这个阈值以适应你的数据
    seg.setInputCloud(bound);
    seg.segment(*inliers, *coefficients);

    PointT circle_center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    double sum_distances = 0.0;
    for (const auto& point : cloud->points) {
        double distance = pcl::euclideanDistance(point, circle_center);
        sum_distances += distance;
    }
    double mean_distance = sum_distances / cloud->points.size();

    CP ret(new CloudT);
    if (mean_distance > coefficients->values[3])
    {
        double ref_distance = coefficients->values[3] + 0.4;
        double tolerance = 0.2;
        for (const auto& point : plane_big->points)
        {
            double distance = std::sqrt(std::pow(point.x - circle_center.x, 2) + std::pow(point.y - circle_center.y, 2) + std::pow(point.z - circle_center.z, 2));
            //cout << distance << endl;
            if (abs(ref_distance - distance) < tolerance)
            {
                ret->push_back(point);
            }
        }
        //Tool::showComparison(plane_big, ret);
    }
    else
    {
        double ref_distance = coefficients->values[3] - 0.4;
        double tolerance = 0.2;
        for (const auto& point : plane_big->points)
        {
            double distance = std::sqrt(std::pow(point.x - circle_center.x, 2) + std::pow(point.y - circle_center.y, 2) + std::pow(point.z - circle_center.z, 2));
            //cout << distance << endl;
            if (abs(ref_distance - distance) < tolerance)
            {
                ret->push_back(point);
            }
        }
        //Tool::showComparison(plane_big, ret);
    }


    //// 创建可视化对象
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //viewer->setBackgroundColor(0, 0, 0);

    //// 添加原始点云
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 255, 255); // 白色点云
    //viewer->addPointCloud<pcl::PointXYZ>(bound, cloud_color, "original cloud");

    //// 提取拟合圆的参数
    //float center_x = coefficients->values[0];
    //float center_y = coefficients->values[1];
    //float center_z = coefficients->values[2];
    //float radius = coefficients->values[3];
    //float normal_x = coefficients->values[4];
    //float normal_y = coefficients->values[5];
    //float normal_z = coefficients->values[6];

    //// 绘制圆
    //pcl::PointXYZ center(center_x, center_y, center_z);
    //pcl::PointXYZ normal(normal_x, normal_y, normal_z);

    //// 创建一个表示圆的点云
    //pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>());
    //int num_points = 10000;
    //for (int i = 0; i < num_points; ++i) {
    //    float angle = 2.0 * M_PI * float(i) / float(num_points);
    //    float x = center_x + radius * cos(angle);
    //    float y = center_y + radius * sin(angle);
    //    float z = center_z;  // 如果圆不在XZ平面，需要将法线方向考虑进来

    //    // 调整z的高度以使圆在其法线方向上
    //    Eigen::Vector3f point(x, y, z);
    //    Eigen::Vector3f axis(0, 0, 1); // 假设圆在XZ平面上
    //    Eigen::Vector3f normal(normal_x, normal_y, normal_z);
    //    Eigen::Quaternionf q;
    //    q.setFromTwoVectors(axis, normal);
    //    point = q * (point - center.getVector3fMap()) + center.getVector3fMap();

    //    circle_points->points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    //}
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> circle_color(circle_points, 255, 0, 0); // 红色圆
    //viewer->addPointCloud<pcl::PointXYZ>(circle_points, circle_color, "fitted circle");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "fitted circle");

    //while (!viewer->wasStopped()) {
    //    viewer->spinOnce(100);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //}


    return ret;
}

CP planeExtract(CP cloud)
{
    CzxTimer asdgsdgijsagi(__func__);

    float tolerance = 0.5;

    arsenal::passThrough(cloud, "z", stof(conf["z"]) - 2, stof(conf["z"]) + 2);
    auto plane_big = maxCluster(cloud);



    PointT center;
    center.getVector3fMap() = plane_big->getMatrixXfMap(3, 4, 0).rowwise().mean();



    double total_distance = 0.0;
    for (const auto& point : plane_big->points)
    {
        double distance = std::sqrt(std::pow(point.x - center.x, 2) + std::pow(point.y - center.y, 2) + std::pow(point.z - center.z, 2));
        total_distance += distance;
    }
    double mean_distance = total_distance / plane_big->points.size();

    CP ret(new CloudT);
    for (const auto& point : plane_big->points)
    {
        double distance = std::sqrt(std::pow(point.x - center.x, 2) + std::pow(point.y - center.y, 2) + std::pow(point.z - center.z, 2));
        //cout << distance << endl;
        if (abs(mean_distance - distance) < tolerance)
        {
            ret->push_back(point);
        }
    }

    int radius = 5;
    float resolution = stof(conf["resolution"]);
    ret = KNNBoundRemove(ret, radius * resolution, 0.8* radius * radius * 3.14);
    //ret = KNNBoundRemove(ret, radius * resolution, 0.8* radius * radius  * 3.14);

    //Tool::showComparison(cloud, ret);
    return ret;
}

void main()
{
    CP cloud(new CloudT);
    {
        pcl::io::loadPCDFile("plane.pcd", *cloud);
        PointT max_point, min_point;
        auto flat = Flatness(cloud, min_point, max_point);
    }
    auto file_list = czx_file::pathGather(conf["root"], "*.ply");
    
    for (auto& path : file_list)
    {
        cout << path << endl;
        {
            CzxTimer _("load");
            pcl::io::loadPLYFile(path, *cloud);
        }
        cloud = planeExtractCircle(cloud);
        PointT max_point, min_point;
        auto flat = Flatness(cloud, min_point, max_point);
        cout << flat << endl;
        file << path << "," << flat << "\n";
    }
    //CP top(new CloudT);
    //top->push_back(min_point);
    //top->push_back(max_point);
    //Tool::showComparison(cloud, top);
}

