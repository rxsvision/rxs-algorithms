#include"../czxDependence/czxTool.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include<pcl/common/common.h>
#include"../czxDependence/2D/ParameterTuner.h"
#include <pcl/common/pca.h>

float calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) +
        std::pow(p1.y - p2.y, 2) +
        std::pow(p1.z - p2.z, 2));
}

CP getGlue(CP cloud, float plane_z, PointT glue_pos)
{
    {
        CzxComparison _(cloud);
        arsenal::passThrough(cloud, "z", plane_z, 9999.0f);
    }
    

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(10000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    float min_dis = 999999;
    CP nearest_cloud(new CloudT);
    for (const auto& indices : cluster_indices) 
    {
        CP cloud_cluster(new CloudT);
        PointT ref_point = cloud->points[indices.indices[0]];
        float ref_dis = calculateDistance(ref_point, glue_pos);
        //if (ref_dis > 5) continue;
        for (const auto& idx : indices.indices)
        {
            cloud_cluster->points.push_back(cloud->points[idx]);
        }
        //Tool::showComparison(cloud, cloud_cluster);
        //float x_dis = (cloud_cluster->getMatrixXfMap(3, 4, 0).row(0).array() - glue_pos.x).mean();
        //float y_dis = (cloud_cluster->getMatrixXfMap(3, 4, 0).row(0).array() - glue_pos.y).mean();
        //float z_dis = (cloud_cluster->getMatrixXfMap(3, 4, 0).row(0).array() - glue_pos.z).mean();

        //float sum_dis = x_dis + y_dis + z_dis;
        if (ref_dis < min_dis)
        {
            min_dis = ref_dis;
            nearest_cloud = cloud_cluster;
        }
    }

    //Tool::showComparison(cloud, nearest_cloud);
    return nearest_cloud;
}


double volume(CP cloud, double xyInterval, float plane_pos)
{
    double sum = 0;
    for (auto p : *cloud)
    {
        sum += xyInterval * xyInterval * (p.z-plane_pos);
    }
    return sum;
}

CP img2pcl(cv::Mat img, float off_x, float off_y)
{
    CP ret(new CloudT);

    for (int r = 0; r < img.rows; r++)
    {
        for (int c = 0; c < img.cols; c++)
        {
            float z = img.at<float>(r, c);
            if (z == 0)continue;
            PointT p(c * off_x, r * off_y, z);
            ret->push_back(p);
        }
    }
    return ret;
}


//CCP img2pcl(cv::Mat img, float off_x, float off_y, cv::Mat color)
//{
//    CCP ret(new CloudCT);
//    if (img.type() != CV_16UC1) {
//        std::cout << "图像不是 16 位单通道格式！" << std::endl;
//        return CCP();
//    }
//
//    for (int r = 0; r < img.rows; r++)
//    {
//        for (int c = 0; c < img.cols; c++)
//        {
//            ushort z_16 = img.at<ushort>(r, c);
//            if (z_16 == 0) continue;
//            float z = z_16 * 0.0004 - 13;
//            PointCT p(c * off_x, r * off_y, z, color.at<uchar>(r, c), color.at<uchar>(r, c), color.at<uchar>(r, c));
//            ret->push_back(p);
//        }
//    }
//    return ret;
//}

cv::Mat eliminateBound(cv::Mat img)
{
    cv::Mat ret = cv::Mat::zeros(img.size(), CV_8UC1);
    int range = 10;
    for (int r = range; r < img.rows - range; r++)
    {
        for (int c = range; c < img.cols - range; c++)
        {
            uchar pixel = img.at<uchar>(r, c);
            if (pixel > 25) continue;

            if (img.at<uchar>(r + range, c) > 55 ||
                img.at<uchar>(r + range-1, c) > 55 ||
                img.at<uchar>(r, c + range) > 55 ||
                img.at<uchar>(r, c + range-1) > 55 ||
                img.at<uchar>(r - range, c) > 55 ||
                img.at<uchar>(r - range+1, c) > 55 ||
                img.at<uchar>(r, c - range) > 55 ||
                img.at<uchar>(r, c - range+1) > 55
                )
            {
                ret.at<uchar>(r, c) = 255;
            }

        }
    }

    //for (int r = range; r < ret.rows - range; r++)
    //{
    //    for (int c = range; c < ret.cols - range; c++)
    //    {
    //        uchar pixel = img.at<uchar>(r, c);
    //        if (pixel > 25) continue;

    //        if ((ret.at<uchar>(r + range, c + range) > 55 && ret.at<uchar>(r - range, c - range) > 55) ||
    //            ret.at<uchar>(r - range, c+range) > 55 && ret.at<uchar>(r+range, c - range) > 55)
    //        {
    //            ret.at<uchar>(r, c) = 255;
    //        }
    //    }
    //}

    return ret;
}

CCP img2pcl_mask(cv::Mat img, float off_x, float off_y, cv::Mat color, cv::Mat mask)
{
    CCP ret(new CloudCT);

    for (int r = 0; r < img.rows; r++)
    {
        for (int c = 0; c < img.cols; c++)
        {
            if (mask.at<uchar>(r, c) == 255) continue;
            float z = img.at<float>(r, c);
            if (z == 0) continue;
            //float z = z_16 * 0.0004 - 13;
            PointCT p(c * off_x, r * off_y, z, color.at<uchar>(r, c), color.at<uchar>(r, c), color.at<uchar>(r, c));
            ret->push_back(p);
        }
    }
    return ret;
}

cv::Mat getGlueMask(cv::Mat gray_img)
{
    cv::Mat kernel_rect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(gray_img, gray_img, kernel_rect);
    cv::erode(gray_img, gray_img, kernel_rect);
    cv::Mat mask;
    cv::threshold(gray_img, mask, 20, 255, cv::THRESH_BINARY_INV);
    showImg(mask);
    return mask;
}

cv::Mat sxTiff2Depth(cv::Mat img, float z_ratio)
{
    cv::Mat new_img;
    img.convertTo(new_img, CV_32FC1);
    new_img = new_img * z_ratio;
    return new_img;
}

template <typename T>
vector<T> eightCorner(cv::Mat img, int r, int c, int off)
{
    vector<T> ret;
    ret.push_back(img.at<T>(r+off, c));
    ret.push_back(img.at<T>(r-off, c));
    ret.push_back(img.at<T>(r+off, c+off));
    ret.push_back(img.at<T>(r-off, c+off));
    ret.push_back(img.at<T>(r+off, c-off));
    ret.push_back(img.at<T>(r-off, c-off));
    ret.push_back(img.at<T>(r, c+off));
    ret.push_back(img.at<T>(r, c-off));

    return ret;
}

template <typename T>
vector<T> fourCorner(cv::Mat img, int r, int c, int off)
{
    vector<T> ret;
    ret.push_back(img.at<T>(r + off, c));
    ret.push_back(img.at<T>(r - off, c));
    ret.push_back(img.at<T>(r, c + off));
    ret.push_back(img.at<T>(r, c - off));

    return ret;
}

template <typename T>
T calculateRange(const std::vector<T>& vec) {
    if (vec.empty()) return T();  // 处理空向量情况，返回默认值
    // 使用 std::minmax_element 获取结果
    auto minmax = std::minmax_element(vec.begin(), vec.end());
    T min_value = *minmax.first;   // 最小值
    T max_value = *minmax.second;  // 最大值

    return max_value - min_value;
}

cv::Mat plane_mask(cv::Mat depth_img)
{
    cv::Mat mask = cv::Mat::zeros(depth_img.size(), CV_8UC1);
    int range = 3;
    for (int r = range; r < depth_img.rows - range; r++)
    {
        for (int c = range; c < depth_img.cols - range; c++)
        {
            auto corners = fourCorner<float>(depth_img, r, c, range);
            //float range = calculateRange(val);
            float pixel = depth_img.at<float>(r, c);
            float max_dif = 0;
            for (auto& corner : corners)
            {
                float dif = abs(pixel - corner);
                if (max_dif < dif)
                    max_dif = dif;
            }
            if (max_dif < 0.003)
                mask.at<char>(r, c) = 255;
        }
    }

    // 定义核大小
    int kernelSize = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::dilate(mask, mask, element);
    cv::erode(mask, mask, element);
    
    return mask;
}

void main()
{
    auto conf = czxTool::readProfile("glue.czx");
    auto gray_img = cv::imread(conf["2Dpath"], cv::IMREAD_GRAYSCALE);
    cv::Mat depth_image = cv::imread(conf["deep_img"], cv::IMREAD_UNCHANGED);
    cout << depth_image.type() << endl;
    //depth_image = sxTiff2Depth(depth_image, 0.0004);
    depth_image = sxTiff2Depth(depth_image, 0.0001);
    //getGlueMask(gray_img);

    cv::Mat binaryImage;
    cv::threshold(gray_img, binaryImage, 40, 255, cv::THRESH_BINARY_INV);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(10, 10));
    cv::erode(binaryImage, binaryImage, kernel);
    cv::dilate(binaryImage, binaryImage, kernel);
    //showImg(binaryImage);

    //auto mask_bound = eliminateBound(gray_img);
    //auto mask_plane = plane_mask(depth_image);
    //cv::Mat mask = mask_plane + mask_bound;
    //showImg(mask_plane);

    //cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5));
    //cv::dilate(mask, mask, kernel);
    //cv::dilate(mask, mask, kernel);
    //cv::erode(mask, mask, kernel);
    //cv::erode(mask, mask, kernel);
    //cv::erode(mask, mask, kernel);
    ////cv::erode(mask, mask, kernel);
    //showImg(mask);

    auto mask = cv::Mat::zeros(depth_image.size(), CV_8U);

    auto cl = img2pcl_mask(depth_image, 0.004, 0.004, gray_img, mask);
    //auto cl = img2pcl_mask(depth_image, 0.0062, 0.0062, gray_img, mask);
    //auto cl = img2pcl(depth_image, 0.004, 0.004);
    pcl::io::savePCDFileBinary("processed.pcd", *cl);

    //BoxImageProcessor bip(20, 20);
    //auto rang_img = bip.processImage(gray_img, BoxOperator::std);
    //showImg(rang_img);


    //BloBParameterTuner bpt(gray_img, mask);
    //bpt.run();

	CP cloud(new CloudT);
	CP origin(new CloudT);
	pcl::io::loadPCDFile("processed.pcd", *cloud);
	*origin = *cloud;

    //CP plane(new CloudT);
    //*plane = *cloud;
    //arsenal::passThrough(plane, 3.6, 3.7, 6.3, 6.4);
    //Tool::showComparison(cloud, plane);    
    //pcl::PCA<pcl::PointXYZ> pca;
    //pca.setInputCloud(cloud);
    //Eigen::Matrix3f eigenvectors = pca.getEigenVectors().cast<float>();
    //Eigen::Vector3f normal = eigenvectors.col(2);

    //float ref_z = 12.7;
    //PointT glue_pos(10, 3, 0);

    //PointT ref_point(4.01, 6.20, 13.17);
    //float ref_z = ref_point.getVector3fMap().dot(normal);
    //cloud->getMatrixXfMap(3, 4, 0).row(2) = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    //float ref_z = 13.17;
    //PointT glue_pos(4,5,13);

    float ref_z = 2.5;
    PointT glue_pos(5,3,2);

    //float ref_z = 2.08;
    //PointT glue_pos(11,13,2);
    auto glue = getGlue(cloud, ref_z, glue_pos);
    float glue_volume = volume(glue, 0.004, ref_z-0.05);
    cout << glue_volume << "立方毫米" << endl;
    //origin->getMatrixXfMap(3, 4, 0).row(2) = normal.transpose() * origin->getMatrixXfMap(3, 4, 0);
    Tool::showComparison(origin, glue);
}