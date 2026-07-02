//#include"czxTool.h"
#include"czxDependence/czxTool.h"
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/filters/passthrough.h>
#include <iostream>
#include <fstream>
#include <pcl/common/distances.h>
#include <pcl/common/angles.h>
#include<pcl/common/pca.h>
#include"SquareTree.h"
#include"utlis.hpp"


//#define CZX_DEBUG
#define cp(i) cloud->points[i]
#define difY(i,j) abs(cp(i).y-cp(j).y)
#define dif(i,j) abs(cp(i).y - cp(j).y)/abs(cp(i).x - cp(j).x)

float dif_y(PointT a, PointT b)
{
    return abs(a.y - b.y);
}

bool verify_y(PointT a, PointT b, float threshold = 0.005)
{
    return dif_y(a, b) < threshold;
}

//边界两团聚集的噪声
CP filterCurveNoise(CP cloud)
{
    //cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));
    //cloud->getMatrixXfMap(3, 4, 0).row(1) *= -1;
    CP ret(new CloudT);
    int i;
    for (i = 1; i < cloud->size(); i++)
    {
        if (cp(i).x - cp(i - 1).x > 0.5)
            break;
    }

    float ref_y = cp(i).y;
    for (; i < cloud->size() - 1; i++)
    {
        if (abs(cp(i).y - ref_y) < 1)
        {
            ref_y = cp(i).y;
            ret->push_back(cp(i));
        }
    }
    #ifdef RXS_HAS_VISUALIZATION
    Tool::showComparison(cloud, ret);
    #endif
    return ret;
}

//曲线四处都有的噪声
CP filterCurveNoise_V2(CP cloud)
{
    CP ret(new CloudT);
    int i;
    for (i = cloud->size()/2; i < cloud->size(); i++)
    {
        if (cp(i).x - cp(i + 1).x < 0.06)
            break;
    }

    float ref_y_mid = cp(i).y;
    PointT ref_p = cp(i);
    PointT mid_p = cp(i);
    float ref_y = ref_y_mid;
    int mid_i = i;
    for (; i < cloud->size(); i++)
    {
        //if ((i!=cloud->size()-1 &&difY(i, i + 1) > 0.08) && difY(i, i - 1) > 0.08)
        //    continue;
        if (cp(i).y < -2.6)
            continue;
        if (pcl::euclideanDistance(cp(i), ref_p) < 0.5)
        {
            ref_y = cp(i).y;
            ret->push_back(cp(i));
            ref_p = cp(i);
        }
    }
    ref_y = ref_y_mid;
    ref_p = mid_p;
    for (i = mid_i; i >= 0; i--)
    {
        if (cp(i).y < -2.6)
            continue;
        if (pcl::euclideanDistance(cp(i), ref_p) < 0.5)
        {
            ref_y = cp(i).y;
            ret->push_back(cp(i));
            ref_p = cp(i);
        }
    }
    sort(ret->begin(), ret->end(), [](PointT a, PointT b) {return a.x < b.x; });

    //Tool::showComparison(cloud, ret);
    return ret;
}

CP preProcessForDebug(CP cloud)
{
    return filterCurveNoise_V2(cloud);
    CP ret(new CloudT);
    //cloud->getMatrixXfMap(3, 4, 0).row(1).swap(cloud->getMatrixXfMap(3, 4, 0).row(2));
    //cloud->getMatrixXfMap(3, 4, 0).row(1) *= -1;

    for (int i = 0; i < cloud->size(); i++)
    {
        auto& p = cloud->points[i];
        if (p.y == 0)
            continue;
        if (i > 1 && i < cloud->size() - 2)
        {
            if (difY(i, i - 1) > 0.06 && difY(i, i + 1) > 0.06)
                continue;
        }
        ret->push_back(p);
    }

    return ret;
}

void comparisonKeyence(CP c1, CP c2, float threshold)
{
    threshold = threshold * threshold;
    CP dif1to2(new CloudT), dif2to1(new CloudT);
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(c1);

            for (int i = 0; i < c2->size(); i++)
            {
                // 执行KNN搜索
                int k = 1; // 选择k值
                std::vector<int> point_indices(k);
                std::vector<float> point_distances(k);
                kdtree.nearestKSearch(c2->points[i], k, point_indices, point_distances);
                if (point_distances[0] > threshold)
                {
                    dif2to1->push_back(c2->points[i]);
                }
            }
        }

        //#pragma omp section
        //{
        //    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        //    kdtree.setInputCloud(c2);

        //    for (int i = 0; i < c1->size(); i++)
        //    {
        //        // 执行KNN搜索
        //        int k = 1; // 选择k值
        //        std::vector<int> point_indices(k);
        //        std::vector<float> point_distances(k);
        //        kdtree.nearestKSearch(c1->points[i], k, point_indices, point_distances);
        //        if (point_distances[0] > threshold)
        //        {
        //            dif1to2->push_back(c1->points[i]);
        //        }
        //    }

        //}
    }

    if (dif2to1->size() > 0)
    {
        #ifdef RXS_HAS_VISUALIZATION
        Tool::showComparison(c1, c2, dif2to1, 2, 2, 5);
        #endif
    }
    //if (dif1to2->size() > 0)
    //{
    //    Tool::showComparison(c1, c2, dif1to2, 2, 2, 5);
    //}

}

//void filter(CP cloud)
//{
//    int index = 0;
//    for (; index < cloud->size()-1; index++)
//    {
//        if(pcl::ecloud->points[i+1])
//    }
//}

//滤掉前后直线和跳变点
void filter(CP cloud)
{
    //CzxComparison sagf(cloud);
    int cur_index = 0;
    double dif_y = 0.02;
    for (; cur_index < cloud->size(); cur_index++)
    {

        if (abs(cloud->points[cur_index + 1].y - cloud->points[cur_index].y) > dif_y || cloud->points[cur_index + 1].y == cloud->points[cur_index].y)
            continue;
        else
            break;
    }
    if (cur_index>1 && abs(cloud->points[cur_index - 1].y - cloud->points[cur_index].y) > dif_y)
        cur_index--;
    cloud->erase(cloud->begin(), cloud->begin() + cur_index);
    if (cloud->size() < 100) return;
    cur_index = cloud->size() - 2;
    for (; cur_index > 0; cur_index--)
    {
        if (abs(cloud->points[cur_index - 1].y - cloud->points[cur_index].y) > dif_y || cloud->points[cur_index - 1].y == cloud->points[cur_index].y)
            continue;
        else
            break;
    }
    if (cur_index<cloud->size() - 2 && abs(cloud->points[cur_index + 1].y - cloud->points[cur_index].y) > dif_y)
    {
        cur_index++;
        //cout << cur_index;
    }
    if (cur_index != cloud->size() - 2)
        cloud->erase(cloud->begin() + cur_index-1, cloud->end());
    if (cloud->size() < 100) return;
}

unordered_map<string, string> conf = czxTool::readProfile("conf_curve.czx");

std::ofstream file(conf["data"], std::ios::trunc);
std::ofstream file_sum("log/summary.csv", std::ios::trunc);

typedef vector<double>(__cdecl* VCVVB)(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool visual);
VCVVB fitBSpline;

template <typename T> vector<double> LoadBSpline(T cloud)
{
    return fitBSpline(cloud, false);
}
template <typename T> vector<double> BigHoleBSpline(T cloud)
{
    double dif = abs(cloud->points[cloud->size() - 1].x - cloud->points[0].x);
    MatrixXf cloud_mat = cloud->getMatrixXfMap(3, 4, 0);
    CP left(new CloudT);
    CP right(new CloudT);
    left->resize(0);
    right->resize(0);

    for (int i = 0; i < cloud->size(); i++)
    {
        if (abs(cloud->points[i+1].x - cloud->points[i].x) > 100.0)
        {
            i++;
            left->resize(i);
            left->getMatrixXfMap(3, 4, 0) = cloud_mat.block(0, 0, 3, i);
            right->resize(cloud->size() - i);
            right->getMatrixXfMap(3, 4, 0) = cloud_mat.block(0, i, 3, cloud->size()-i);
            break;
        }
    }
    if (left->size() == 0 || right->size() == 0)
        return LoadBSpline(cloud);
    //Tool::showComparison(left, right, cloud,2,2,1);


    auto ret_left = fitBSpline(left, false);
    auto ret_right = fitBSpline(right, false);

    *cloud = *left + *right;
    return { ret_left[0]+ ret_right[0]+pcl::euclideanDistance(left->points[0], right->points[right->size()-1]),
        dif,
        max(ret_left[2], ret_right[2])};
}
template <typename T> vector<double> BSpline(T cloud)
{    
    return LoadBSpline(cloud);
    //return BigHoleBSpline(cloud);
}

double Length2D(CP cloud)
{
    {
        //CzxComparison _(cloud);
        arsenal::passThrough(cloud, "y", 4, 999);
    }
    bool greater = true;
    if (cloud->size() < 10)
    {
        cout << "y值高于4的点数目过少" << endl;
        return -1;
    }
    
    if (cloud->points[0].x > cloud->points[1].x)
        greater = false;
    for (int i=1;i<cloud->size();i++)
    {
        if (abs(cloud->points[i].x - cloud->points[i - 1].x) > 50)
            return pcl::euclideanDistance(cloud->points[i], cloud->points[i - 1]);
    }

    cout << "找不到内侧点" << endl;
    return -1;
}

//滤掉前后直线
CP filterLine(CP cloud)
{
    double dif_y = 0.001;
    int star_index = 0;
    for (; star_index < cloud->size()-2; star_index++)
    {
        if (abs(cloud->points[star_index + 1].y - cloud->points[star_index].y) > dif_y&& abs(cloud->points[star_index + 1].y - cloud->points[star_index+2].y) > dif_y&& abs(cloud->points[star_index + 3].y - cloud->points[star_index + 2].y) > dif_y)
            break;
    }
    if(star_index!=0)
        star_index++;
    int end_index = cloud->size() - 1;
    for (; end_index > 2; end_index--)
    {
        if (abs(cloud->points[end_index].y - cloud->points[end_index -1].y) > dif_y&& abs(cloud->points[end_index-2].y - cloud->points[end_index - 1].y) > dif_y && abs(cloud->points[end_index - 2].y - cloud->points[end_index - 3].y) > dif_y)
            break;
    }
    if(end_index!= cloud->size() - 1)
        end_index--;

    CP ret(new CloudT);
    for (int i = star_index; i <= end_index; i++)
    {
        ret->push_back(cloud->points[i]);
    }
    return ret;
}

CP FilterByTruningPoint(CP cloud, int radius)
{
    //CP cloud = filterLine(cloud_in);
    int star_index, end_index;
    float angle_threshold = stof(conf["angle"]);
    float angle_threshold_global = stof(conf["angle_global"]);
    float angle;

    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    Vector3f normal_global = pca.getEigenVectors().col(1);

    for (star_index = radius; star_index < cloud->size() - radius; star_index++)
    {
        //index点在右点云中

        CP left(new CloudT);
        left->resize(radius);
        left->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, star_index - radius, 3, radius);
        pcl::PCA<PointT> pca;
        pca.setInputCloud(left);
        Vector3f normal_left = pca.getEigenVectors().col(1);

        CP right(new CloudT);
        right->resize(radius);
        right->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, star_index, 3, radius);
        pca.setInputCloud(right);
        Vector3f normal_right = pca.getEigenVectors().col(1);

        angle = pcl::getAngle3D(normal_left, normal_right, true);
        double angle_global = pcl::getAngle3D(normal_global, normal_right, true);
        if (angle > 90)
            angle = 180 - angle;
        if (angle_global > 90)
            angle_global = 180 - angle_global;
        #ifdef CZX_DEBUG
        #ifdef RXS_HAS_VISUALIZATION
        Tool::showComparison(cloud, right);
        #endif
        #endif
        if (angle > angle_threshold&& angle_global< angle_threshold_global)
        {
            #ifdef CZX_DEBUG
            #ifdef RXS_HAS_VISUALIZATION
            Tool::showComparison(cloud, right);
            #endif
            #endif
            break;
        }
    }
    if (star_index == radius)
        star_index = 0;


    for (end_index = cloud->size() - radius -1 ; end_index > radius; end_index--)
    {
        //index点在右点云中
        CP left(new CloudT);
        left->resize(radius);
        left->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, end_index - radius, 3, radius);
        pcl::PCA<PointT> pca;
        pca.setInputCloud(left);
        Vector3f normal_left = pca.getEigenVectors().col(1);

        CP right(new CloudT);
        right->resize(radius);
        right->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, end_index, 3, radius);
        pca.setInputCloud(right);
        Vector3f normal_right = pca.getEigenVectors().col(1);

        angle = pcl::getAngle3D(normal_left, normal_right, true);
        double angle_global = pcl::getAngle3D(normal_left, normal_global, true);
        if (angle > 90)
            angle = 180 - angle;
        if (angle_global > 90)
            angle_global = 180 - angle_global;
        if (angle > angle_threshold&& angle_global < angle_threshold_global)
        {
            #ifdef CZX_DEBUG
            #ifdef RXS_HAS_VISUALIZATION
            Tool::showComparison(cloud, left);
            #endif
            #endif
            break;
        }
    }
    if (end_index == cloud->size() - radius - 1)
        end_index = cloud->size() - 1;

    CP ret(new CloudT);
    for (int i = star_index; i <= end_index; i++)
    {
        ret->push_back(cloud->points[i]);
    }

    return ret;
}

//滤掉logo交界处的洞
CP filterHole(CP cloud)
{
    int compare_radius = 10;
    int star_index=0, end_index=0;
    int count = 0;
    for (int i = compare_radius; i < cloud->size() - compare_radius; i++)
    {
        if (cloud->points[i].y - cloud->points[i + compare_radius].y < -0.1 && cloud->points[i].y - cloud->points[i - compare_radius].y < -0.1)
        {
            count++;
        }
        else
        {
            if (count > 3)
            {
                end_index = i - 1;
                star_index = i - count;
                break;
            }
            else
            {
                count = 0;
            }
        }
    }
    if (star_index == 0)
    {
        return cloud;
    }
    CP ret(new CloudT);
    for (int i = 0; i < cloud->size(); i++)
    {
        if (i > end_index || i < star_index)
        {
            ret->push_back(cloud->points[i]);
        }
    }
    return ret;
}

//根据跳变获取边界点
double Length2D_V2(CP cloud, CP boundary)
{
    #define OUTSIDE
    PointT left, right;
    int i;
    for (i = 0; i < cloud->size()-1; i++)
    {
        if (cloud->points[i].y - cloud->points[i + 1].y > 0.5
            //&& cloud->points[i].y - cloud->points[i + 2].y > 0.5
            )
        {
            break;
        }
    }
    //#ifdef OUTSIDE
    ////用来处理弧形边界问题
    //while (i != cloud->size() - 1 && difY(i+1, i + 10) > 0.06)
    //    i++;
    //#endif

    if (i == cloud->size() - 1)
        cout << "error" << endl;
    #ifdef OUTSIDE
    left = cloud->points[i + 1];
    #else
    left = cloud->points[i];
    #endif


    for (i = cloud->size()-1; i > 1; i--)
    {
        if (cloud->points[i].y - cloud->points[i - 1].y > 0.5)
        {

            break;
        }
    }

    //#ifdef OUTSIDE
    ////用来处理弧形边界问题
    //while (i != 1 && difY(i - 1, i - 10) > 0.06)
    //    i--;
    //#endif

    if (i == 1)
    {
        cout << "error" << endl;
    }
    #ifdef OUTSIDE
    right = cloud->points[i - 1];
    #else
    right = cloud->points[i];
    #endif
    boundary->push_back(left);
    boundary->push_back(right);

    //Tool::showComparison(cloud, boundary);
    return pcl::euclideanDistance(left, right);
}

//滤掉超角度量程的点云
CP filterAngle(CP cloud, double angle)
{
    int range = 5;
    int star_index = 0, end_index = cloud->size() - 1;

    pcl::PCA<PointT> pca;
    CP tmp(new CloudT);
    tmp->resize(range);
    for (; star_index < cloud->size() - range; star_index++)
    {
        tmp->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, star_index, 3, range);
        pca.setInputCloud(tmp);

        Eigen::Vector3f x_axis(0.0, 1.0, 0.0);

        auto evs = pca.getEigenVectors();

        float angle_rad = pcl::getAngle3D(evs.col(0), x_axis, true);
        if (angle_rad < 0)
        {
            angle_rad = -angle_rad;
        }
        if (angle_rad > angle && angle_rad < (180 - angle))
            break;
    }

    for (; end_index > range; end_index--)
    {
        tmp->getMatrixXfMap(3, 4, 0) = cloud->getMatrixXfMap(3, 4, 0).block(0, end_index - range, 3, range);
        pca.setInputCloud(tmp);

        Eigen::Vector3f x_axis(0.0, 1.0, 0.0);

        float angle_rad = pcl::getAngle3D(pca.getEigenVectors().col(0), x_axis, true);
        if (angle_rad < 0)
        {
            angle_rad = -angle_rad;
        }
        if (angle_rad > angle && angle_rad < (180 - angle))
            break;
    }

    CP ret(new CloudT);
    for (int i = 0; i < cloud->size(); i++)
    {
        if (i < end_index && i > star_index)
        {
            ret->push_back(cloud->points[i]);
        }
    }
    return ret;
}

Matrix3f PCAModelize(CP cloud)
{
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    auto projection = pca.getEigenVectors().transpose();

    cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);
    return projection;
}

PointT getCenter(CP cloud)
{
    CP tmp(new CloudT);
    for (int i = 20; i < cloud->size() - 20; i++)
    {
        tmp->push_back(cloud->points[i]);
    }
    PointT center;
    center.getVector3fMap() = tmp->getMatrixXfMap(3, 4, 0).rowwise().mean();
    center.x = (cloud->points[0].x + cloud->points[cloud->size() - 1].x) / 2;
    return center;
}

double getMaxDifY(CP source, CP target)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target);

    #ifdef CZX_DEBUG
    int max_source, max_target;
    #endif

    double max_dif_y = 0;

    for (int i = 0; i < source->size(); i++)
    {
        // 执行KNN搜索
        int k = 1; // 选择k值
        std::vector<int> point_indices(k);
        std::vector<float> point_distances(k);
        kdtree.nearestKSearch(source->points[i], k, point_indices, point_distances);
        //if (max_dis < point_distances[0])
        //{
        //	max_dis = point_distances[0];
        //	#ifdef CZX_DEBUG
        //	max_source = i;
        //	max_target = point_indices[0];
        //	#endif
        //}
        double dif_y = source->points[i].y - target->points[point_indices[0]].y;
        if (dif_y > max_dif_y)
        {
            max_dif_y = dif_y;
        }
    }
    return max_dif_y;
}

vector<double> comparisonTwoCloud(CP source, CP target)
{
    PointT model_left, model_right;
    if (target->points[0].x > target->points[target->size() - 1].x)
    {
        model_left = target->points[target->size() - 1];
        model_right = target->points[0];
    }
    else
    {
        model_right = target->points[target->size() - 1];
        model_left = target->points[0];
    }

    PointT source_left, source_right;
    if (source->points[0].x > source->points[source->size() - 1].x)
    {
        source_left = source->points[source->size() - 1];
        source_right = source->points[0];
    }
    else
    {
        source_right = source->points[source->size() - 1];
        source_left = source->points[0];
    }


    #ifdef CZX_DEBUG
    if (model_left.x - source_left.x < min_dif_debug)
    {
        source->getMatrixXfMap(3, 4, 0).colwise() -= Vector3f(0, max_dif_y, 0);
        #ifdef RXS_HAS_VISUALIZATION
        Tool::showComparison(target, source);
        #endif
    }
    if (model_left.x - source_left.x > max_dif_debug)
    {
        source->getMatrixXfMap(3, 4, 0).colwise() -= Vector3f(0, max_dif_y, 0);
        #ifdef RXS_HAS_VISUALIZATION
        Tool::showComparison(target, source);
        #endif
}
    #endif



    return { model_left.x - source_left.x, model_left.y - source_left.y, model_right.x - source_right.x, model_right.y - source_right.y };
    //return sqrt(max_dis);
}

//用kd树滤掉离散的噪点
CP filterNoise(CP cloud)
{
    float x_radius = 0.02, y_radius = 0.02;
    SquareTree st;
    st.build(cloud);
    float radius[2] = { 20 * x_radius, 40 * y_radius };
    int threshold = 10;
    CP ret(new CloudT);

    {
        int thread_num = 20;
        vector<CP> omp_ret(thread_num);
        for (auto& value : omp_ret)
        {
            value.reset(new CloudT);
        }
        #pragma omp parallel for num_threads(thread_num)
        for (int i = 0; i < cloud->size(); ++i)
        {
            //vector<PointT>* matches = new vector<PointT>();
            CP matches(new CloudT);
            int num_matches = st.squareSearch(cloud->points[i], radius, matches);
            if (num_matches > threshold)
            {
                //ret->push_back(cloud->points[i]);
                omp_ret[omp_get_thread_num()]->push_back(cloud->points[i]);
            }
            //delete matches;
        }
        for (auto& value : omp_ret)
        {
            *ret += *value;
        }
    }

    return ret;
}

CP filterNoiseDouble(CP cloud)
{
    return filterNoise(filterNoise(cloud));
}


//根据跳变滤掉离散的噪点
CP filterNoiseV2(CP cloud)
{
    CP ret(new CloudT);
    int star = 0, end = cloud->size()-1;
    float threshold = 3;
    float threshold2 = 3;
    for (; star < end; star++)
    {
        auto i = star;
        if (dif(i,i+1)<threshold&&dif(i+1,i+2)<threshold && dif(i + 2, i + 3) < threshold && dif(i + 3, i + 4) < threshold && 
            dif(i, i + 2) < threshold2&&dif(i+1,i+3)<threshold2 && dif(i + 2, i + 4) < threshold2 && dif(i + 3, i + 5) < threshold2)
            break;
    }
    for (; end > star; end--)
    {
        auto i = end;
        if (dif(i, i - 1) < threshold && dif(i - 1, i - 2) < threshold && dif(i - 2, i - 3) < threshold && dif(i - 3, i - 4) < threshold &&
            dif(i, i - 2) < threshold2 && dif(i - 1, i - 3) < threshold2 && dif(i - 2, i - 4) < threshold2 && dif(i - 3, i - 5) < threshold2)
            break;
    }
    for (int i = star; i <= end; i++)
    {
        ret->push_back(cp(i));
    }
    return ret;
}

CP filterNoiseV3(CP cloud)
{
    return filterNoiseV2(filterNoise(cloud));
}

void save3D(string root, string file_name, CP filter_func(CP)=nullptr)
{
    //if (file_name.find("3_X") == string::npos) return;


    vector<string> path_list;
    {
        string inPath = root + file_name;
        __int64 handle;
        struct _finddata_t fileinfo;
        handle = _findfirst(inPath.c_str(), &fileinfo);
        if (handle == -1)
            return;

        do
        {
            //找到的文件的文件名
            string szPath;
            szPath = root + string(fileinfo.name);
            path_list.push_back(szPath);
        } while (!_findnext(handle, &fileinfo));
    }

    double max = -1;
    double min = 99999;
    CP max_cloud, max_cloud_in;
    CP min_cloud, min_cloud_in;
    vector<double> ret_max;
    vector<double> ret_min;
    int arg_max = -1;
    int arg_min = -1;

    //omp_set_num_threads(4);
    #ifndef CZX_DEBUG
    #pragma omp parallel for
    #endif // !CZX_DEBUG
    for (int i = 0; i < path_list.size(); i++)
    {
        CP cloud_ori(new CloudT);
        pcl::io::loadPCDFile(path_list[i], *cloud_ori);
        //pcl::PassThrough<PointT> pass;
        //pass.setInputCloud(cloud);
        //pass.setFilterFieldName("y");
        //pass.setFilterLimits(-20, FLT_MAX);
        //pass.filter(*cloud);

        cloud_ori = preProcessForDebug(cloud_ori);

        CP cloud_filtered(new CloudT);
        if (filter_func)
            cloud_filtered = filter_func(cloud_ori);
        else
            *cloud_filtered = *cloud_ori;
        if (conf["root"].find("debug") != std::string::npos)
        {
#ifdef RXS_HAS_VISUALIZATION
            Tool::showComparison(cloud_ori, cloud_filtered);
#endif
        }
        vector<double> ret;
        {
            CzxComparison __(cloud_filtered);
            if (conf["root"].find("debug") == std::string::npos)
                __.visual = false;
            ret = BSpline(cloud_filtered);
        }
        //Tool::showComparison(cloud, cloud_in);

        #pragma omp critical
        {
            file << ",";
            for (const auto& value : ret) {
                file << value << ",";
            }
            file << path_list[i] << ",";
            file << "\n";

            if (ret[0] > max)
            {
                max_cloud = cloud_ori;
                max_cloud_in = cloud_filtered;
                max = ret[0];
                ret_max = ret;
                arg_max = i;
            }
            if (ret[0] < min)
            {
                min_cloud = cloud_ori;
                min_cloud_in = cloud_filtered;
                min = ret[0];
                ret_min = ret;
                arg_min = i;
            }
        }
    }

    {
        file_sum << "max," << ret_max[0] << "," << ret_max[1] << "," << ret_max[2] << "," << path_list[arg_max] << "\n";
        file_sum << "min," << ret_min[0] << "," << ret_min[1] << "," << ret_min[2] << "," << path_list[arg_min] << "\n";
        file_sum << "dif," << ret_max[0] - ret_min[0] << "," << ret_max[1] - ret_min[1] << "\n";
        file_sum << "grr," << (ret_max[0] - ret_min[0]) / 0.2 * 100 << "%" << "," << (ret_max[1] - ret_min[1]) / 0.2 * 100 << "%" << "\n";

        file_sum << "\n";
    }


    file << "max," << ret_max[0] << "," << ret_max[1] << "," << ret_max[2] << "," << path_list[arg_max] << "\n";
    file << "min," << ret_min[0] << "," << ret_min[1] << "," << ret_min[2] << "," << path_list[arg_min] << "\n";
    file << "dif," << ret_max[0] - ret_min[0] << "," << ret_max[1] - ret_min[1] << "\n";
    file << "grr," << (ret_max[0] - ret_min[0]) / 0.2 * 100 << "%" << "," << (ret_max[1] - ret_min[1]) / 0.2 * 100 << "%" << "\n";

    file << "\n";
    file << "\n";

    cout << "dif: " << ret_max[0] - ret_min[0] << endl;
    cout << "2D dif: " << ret_max[1] - ret_min[1] << endl;
    cout << "ratio: " << (ret_max[0] - ret_min[0]) / 0.2 * 100 << "%" << endl;
    cout << "min_path" << path_list[arg_min] << endl;
    cout << "max_path" << path_list[arg_max] << endl;
    if((ret_max[0] - ret_min[0]) / 0.2 * 100 > 25)
        comparisonKeyence(max_cloud, min_cloud, 0.01);
    //if ((ret_max[1] - ret_min[1]) / 0.2 * 100 > 25)
    //    comparisonKeyence(max_cloud, min_cloud, 0.01);
    if ((ret_max[0] - ret_min[0]) / 0.2 * 100 > 25)
        comparisonKeyence(max_cloud_in, min_cloud_in, 0.01);
}

void saveSkewness(string root, string file_name, string model_path="")
{
    //if (file_name.find("3_X") == string::npos) return;


    //#define FILTER
    vector<string> path_list;
    {        
        string inPath = root + file_name;
        __int64 handle;
        struct _finddata_t fileinfo;
        handle = _findfirst(inPath.c_str(), &fileinfo);
        if (handle == -1)
            return ;

        do
        {
            //找到的文件的文件名
            string szPath;
            szPath = root + string(fileinfo.name);
            path_list.push_back(szPath);
        } while (!_findnext(handle, &fileinfo));
    }
    //for (int i = 0; i < path_list.size(); i++)
    //{
    //    CP cloud(new CloudT);
    //    pcl::io::loadPCDFile(path_list[i], *cloud);
    //    CP ret = filterHole(cloud);
    //    //Tool::showComparison(cloud, ret);
    //}

    double max = -1;
    double min = 99999;
    CP max_cloud, max_cloud_in;
    CP min_cloud, min_cloud_in;
    vector<double> ret_max;
    vector<double> ret_min;
    int arg_max = -1;
    int arg_min = -1;


    CP model(new CloudT);
    PointT center_model;
    if (model_path!= "")
    {
        pcl::io::loadPCDFile(model_path, *model);
        sort(model->begin(), model->end(), [](PointT a, PointT b) {return a.x < b.x; });
        model = filterAngle(model, 30);
        PCAModelize(model);
        center_model = getCenter(model);
    }
    Matrix3f projection_scan;
    Vector3f translate_scan;
    {
        CP source(new CloudT);
        pcl::io::loadPCDFile(path_list[0], *source);
        projection_scan = PCAModelize(source);
        PointT center_source = getCenter(source);
        translate_scan = (center_source.getVector3fMap() - center_model.getVector3fMap());
    }

    {
        CP source(new CloudT);
        pcl::io::loadPCDFile(path_list[0], *source);
        source->getMatrixXfMap(3, 4, 0) = projection_scan * source->getMatrixXfMap(3, 4, 0);
        source->getMatrixXfMap(3, 4, 0).colwise() -= translate_scan;
        translate_scan[1] += getMaxDifY(source, model);
    }

    //omp_set_num_threads(4);
    #ifndef CZX_DEBUG
    #pragma omp parallel for
    #endif // !CZX_DEBUG    
    for (int i = 0; i < path_list.size(); i++)
    {
        CP cloud(new CloudT);
        pcl::io::loadPCDFile(path_list[i], *cloud);
        if (file_name.find("1_X") != string::npos)
        {
            cloud = filterHole(cloud);
        }
        //pcl::PassThrough<PointT> pass;
        //pass.setInputCloud(cloud);
        //pass.setFilterFieldName("y");
        //pass.setFilterLimits(-20, FLT_MAX);
        //pass.filter(*cloud);

        //filter(cloud);

        CP cloud_in(new CloudT);
        #ifdef FILTER
        CP cloud_filtered(new CloudT);
        #endif
        *cloud_in = *cloud;
        {
            //#ifdef CZX_DEBUG
            //CzxComparison _(cloud_in);
            //#endif
            #ifdef FILTER
            cloud_in = filterLine(cloud_in);
            //cloud_in = FilterByTruningPoint(cloud_in,stoi(conf["radius"]));
            *cloud_filtered = *cloud_in;
            //filter(cloud_in);
            #endif
        }

        auto ret = BSpline(cloud_in);
        if (model_path!="")
        {
            //PCAModelize(cloud);
            //PointT center_source = getCenter(cloud);
            //cloud->getMatrixXfMap(3, 4, 0).colwise() -= (center_source.getVector3fMap() - center_model.getVector3fMap());

            CP cloud_tmp(new CloudT);
            *cloud_tmp = *cloud;
            cloud_tmp->getMatrixXfMap(3, 4, 0) = projection_scan * cloud_tmp->getMatrixXfMap(3, 4, 0);
            cloud_tmp->getMatrixXfMap(3, 4, 0).colwise() -= translate_scan;

            vector<double> deformation = comparisonTwoCloud(cloud_tmp, model);
            ret.insert(ret.end(), deformation.begin(), deformation.end());
            if (cloud->points[0].x < cloud->points[cloud->size() - 1].x)
            {
                ret.push_back(cloud->points[0].x);
                ret.push_back(cloud->points[0].y);                
                ret.push_back(cloud->points[cloud->size() - 1].x);
                ret.push_back(cloud->points[cloud->size() - 1].y);
            }
            else
            {
                ret.push_back(cloud->points[cloud->size() - 1].x);
                ret.push_back(cloud->points[cloud->size() - 1].y);
                ret.push_back(cloud->points[0].x);
                ret.push_back(cloud->points[0].y);
            }
        }
        //Tool::showComparison(cloud, cloud_in);

        #pragma omp critical
        {
            file << ",";
            for (const auto& value : ret) {
                file << value << ",";
            }
            file << path_list[i] << ",";
            file << "\n";

            if (ret[0] > max)
            {
                max_cloud = cloud;
                #ifdef FILTER
                max_cloud_in = cloud_filtered;
                #endif
                max = ret[0];
                ret_max = ret;
                arg_max = i;
            }
            if (ret[0] < min)
            {
                min_cloud = cloud;
                #ifdef FILTER
                min_cloud_in = cloud_filtered;
                #endif
                min = ret[0];
                ret_min = ret;
                arg_min = i;
            }
        }
    }

    {
        file_sum << "max," << ret_max[0] << "," << ret_max[1] << "," << ret_max[2] << "," << path_list[arg_max] << "\n";
        file_sum << "min," << ret_min[0] << "," << ret_min[1] << "," << ret_min[2] << "," << path_list[arg_min] << "\n";
        file_sum << "dif," << ret_max[0] - ret_min[0] << "," << ret_max[1] - ret_min[1] << "\n";

        file_sum << "\n";
        file_sum << "\n";
        file_sum << "\n";
    }


    file << "max," << ret_max[0] << "," << ret_max[1] << "," << ret_max[2] << "," << path_list[arg_max] <<"\n";
    file << "min," << ret_min[0] << "," << ret_min[1] << "," << ret_min[2] << "," << path_list[arg_min] <<"\n";

    file << "\n";
    file << "\n";
    file << "\n";

    cout << "dif: " << ret_max[0] - ret_min[0] << endl;
    cout << "2D dif: " << ret_max[1] - ret_min[1] << endl;
    cout << "ratio: " << (ret_max[1] - ret_min[1]) / 0.2 * 100 << "%" << endl;
    cout << "min_path" << path_list[arg_min] << endl;
    cout << "max_path" << path_list[arg_max] << endl;
    //if((ret_max[0] - ret_min[0]) / 0.2 * 100 > 25)
    //    comparisonKeyence(max_cloud, min_cloud, 0.01);
    //if ((ret_max[1] - ret_min[1]) / 0.2 * 100 > 25)
    //    comparisonKeyence(max_cloud, min_cloud, 0.01);
    //if ((ret_max[1] - ret_min[1]) / 0.2 * 100 > 25)
    //    comparisonKeyence(max_cloud_in, min_cloud_in, 0.01);
}

CP getMidLine(CP cloud)
{
    int mid = cloud->size() / 2;
    CP ret(new CloudT);
    int star_index, end_index;
    for (star_index = mid; mid > 1; star_index--)
    {
        if (abs(cloud->points[star_index - 1].y - cloud->points[star_index].y) > 0.1)
            break;
    }
    star_index += 20;
    for (end_index = mid; mid <cloud->size()-1; end_index++)
    {
        if (abs(cloud->points[end_index + 1].y - cloud->points[end_index].y) > 0.1)
            break;
    }
    end_index -= 20;
    
    for (int i = star_index; i < end_index; i++)
    {
        ret->push_back(cloud->points[i]);
    }
    if (ret->size() < 100)
        throw logic_error("轮廓度曲线抽取异常");
    return ret;
}

CP processForPrecitec(CP cloud)
{
    CP ret(new CloudT);

    for (int i = 0; i < cloud->size(); i++)
    {
        auto& p = cloud->points[i];
        if (p.y == 0)
            continue;
        if (i > 3 && i < cloud->size() - 4)
        {
            if (difY(i, i - 1) > 0.06 && difY(i, i + 1) > 0.06) //滤除单个跳变点
                continue;
        }

        ret->push_back(p);
    }

    //ret = filterInvalid(ret);

    #ifdef CZX_DEBUG
    #ifdef RXS_HAS_VISUALIZATION
    Tool::showComparison(cloud, ret);
    #endif
    #endif

    return ret;
}

//删除全等直线
CP filterInvalid(CP cloud)
{
    CP ret(new CloudT);
    for (int i = 0; i < cloud->size(); i++)
    {
        int off = 2;
        if (i + 1 < cloud->size() &&
            verify_y(cp(i), cp(i + 1), 1e-6))
        {

            while (i + off < cloud->size() &&
                verify_y(cp(i), cp(i + off), 1e-6))
                off++;
        }
        if (off > 2)
        {
            i += off - 1;
            continue;
        }
        if (i != cloud->size())
            ret->push_back(cp(i));
    }
    //#ifdef CZX_DEBUG
    //Tool::showComparison(cloud, ret);
    //#endif
    return ret;
}

//删除跳变点
CP filterJump(CP cloud)
{
    float jmp_threshold = 0.04;
    CP ret(new CloudT);
    int rang = 1;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (i < rang && difY(i, i + rang) < jmp_threshold)
        {
            ret->push_back(cp(i));
            continue;
        }
        if (i > cloud->size() - rang - 1 && difY(i - rang, i) < jmp_threshold)
        {
            ret->push_back(cp(i));
            continue;
        }
        if (difY(i, i + rang) < jmp_threshold && difY(i - rang, i) < jmp_threshold)
        {
            ret->push_back(cp(i));
            continue;
        }
    }
    #ifdef CZX_DEBUG
    #ifdef RXS_HAS_VISUALIZATION
    Tool::showComparison(cloud, ret);
    #endif
    #endif
    return ret;
}

CP meanSmooth(CP cloud)
{
    #ifdef CZX_DEBUG
    CzxComparison _(cloud);
    #endif
    {
        CircularBuffer buffer(3);
        for (int i = 0; i < cloud->size(); i++)
        {
            buffer.add(cp(i).y);
            cp(i).y = buffer.average();
        }
    }
    {
        CircularBuffer buffer(3);
        for (int i = cloud->size()-1; i >=0; i--)
        {
            buffer.add(cp(i).y);
            cp(i).y = buffer.average();
        }
    }
    return cloud;
}

vector<pair<string, float>> save2D(string root, string file_name, vector<CP(*)(CP)>preProcess = { meanSmooth, filterInvalid,  filterJump })
{

    vector<pair<string, float>> max_min_ret;

    vector<string> path_list;
    {
        string inPath = root + file_name;
        __int64 handle;
        struct _finddata_t fileinfo;
        handle = _findfirst(inPath.c_str(), &fileinfo);
        if (handle == -1)
            return vector<pair<string, float>>();

        do
        {
            //找到的文件的文件名
            string szPath;
            szPath = root + string(fileinfo.name);
            path_list.push_back(szPath);
        } while (!_findnext(handle, &fileinfo));
    }

    double max = -1;
    double min = 99999999999999;
    CP max_cloud;
    CP min_cloud;
    vector<double> ret_max;
    vector<double> ret_min;
    int arg_max = 0;
    int arg_min = 0;

    CP max_boundary(new CloudT), min_boundary(new CloudT);

    #pragma omp parallel for
    for (int i = 0; i < path_list.size(); i++)
    {
        CP cloud(new CloudT);
        pcl::io::loadPCDFile(path_list[i], *cloud);
        if (preProcess.size() != 0)
        {
            for (auto prePro : preProcess)
                cloud = prePro(cloud);
        }
        CP boundary(new CloudT);

        vector<double> ret(1);
        ret[0] = Length2D_V2(cloud, boundary);


        #pragma omp critical
        {
            file << ",";
            for (const auto& value : ret) {
                file << value << ",";
            }
            file << path_list[i];
            file << "\n";

            if (ret[0] > max)
            {
                max_cloud = cloud;
                max = ret[0];
                ret_max = ret;
                arg_max = i;
                max_boundary = boundary;
            }
            if (ret[0] < min)
            {
                min_cloud = cloud;
                min = ret[0];
                ret_min = ret;
                arg_min = i;
                min_boundary = boundary;
            }
        }
    }

    {
        //file_sum << "max,";
        //for (const auto& value : ret_max) {
        //    file_sum << value << ",";
        //}
        //file_sum << path_list[arg_max] << "\n";
        //file_sum << "min,";
        //for (const auto& value : ret_min) {
        //    file_sum << value << ",";
        //}
        //file_sum << path_list[arg_min] << "\n";
        //file_sum << "dif,";
        //for (int index = 0; index < ret_max.size(); index++)
        //{
        //    file_sum << ret_max[index] - ret_min[index] << ",";
        //}
        //file_sum << "\n";

        //file_sum << "\n";
        //file_sum << "\n";

        pair<string, float> min_val_path;
        pair<string, float> max_val_path;
        min_val_path.first = path_list[arg_min];
        min_val_path.second = ret_min[0];
        max_val_path.first = path_list[arg_max];
        max_val_path.second = ret_max[0];
        max_min_ret.push_back(max_val_path);
        max_min_ret.push_back(min_val_path);
    }

    {
        file << "max,";
        for (const auto& value : ret_max) {
            file << value << ",";
        }
        file << path_list[arg_max] << "\n";
        file << "min,";
        for (const auto& value : ret_min) {
            file << value << ",";
        }
        file << path_list[arg_min] << "\n";
        file << "dif,";
        for (int index = 0; index < ret_max.size(); index++)
        {
            file << ret_max[index] - ret_min[index] << ",";
        }
        file << "\n";

        file << "\n";
        file << "\n";
    }

    cout << "dif: " << ret_max[0] - ret_min[0] << endl;
    //cout << "2D dif: " << ret_max[1] - ret_min[1] << endl;
    cout << "grr: " << (ret_max[0] - ret_min[0]) / 0.2 * 100 << "%" << endl;
    cout << "min_path" << path_list[arg_min] << endl;
    cout << "max_path" << path_list[arg_max] << endl;
    //if ((ret_max[0] - ret_min[0]) / 0.2 * 100 > 25)
    //    comparisonKeyence(max_cloud, min_cloud, 0.01);
    *max_boundary += *min_boundary;
    if ((ret_max[0] - ret_min[0]) / 0.2 * 100 > 20)
    {
        #ifdef RXS_HAS_VISUALIZATION
        Tool::showComparison(max_cloud, min_cloud, max_boundary,1,1,3);
        #endif
    }
    //if ((ret_max[1] - ret_min[1]) / 0.2 * 100 > 25)
    //    comparisonKeyence(max_cloud, min_cloud, 0.01);
    return max_min_ret;
}

std::vector<std::string> splitString(const std::string& input, char delimiter = ' ') {
    std::vector<std::string> result;
    std::istringstream iss(input);
    std::string token;

    while (std::getline(iss, token, delimiter)) {
        result.push_back(token);
    }

    // 如果字符串中没有分隔符，也将整个字符串作为一个元素加入结果
    if (result.empty()) {
        result.push_back(input);
    }

    return result;
}

int main(int argc, char* argv[])
{
    //if (argc != 3) {
    //    std::cerr << "需要且只能两个文件" << std::endl;
    //    return 1;
    //}

    //CP c1(new CloudT), c2(new CloudT);
    //pcl::io::loadPCDFile(argv[1], *c1);
    //pcl::io::loadPCDFile(argv[2], *c2);

    //comparisonKeyence(c1, c2, 0.01);

    HMODULE hDLL = LoadLibraryW(L"czxToolkit.dll");
    typedef vector<double>(__cdecl* VCVVB)(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool visual);
    fitBSpline = (VCVVB)GetProcAddress(hDLL,"fitBSpline_");

    vector<vector<pair<string, float>>> ret_total;

    //auto conf = czxTool::readProfile("deform_conf.czx");
    auto path_list = arsenal::getSubdirectories(conf["big_root"]);
    for (auto root : path_list)
    {
        root = root + "//";
        std::time_t currentTime = std::time(nullptr);
        // 将时间转换为字符串
        char timeString[100];
        std::strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", std::localtime(&currentTime));


        //file_sum << conf["root"] << "," << conf["note"] << "," << timeString << "\n";

        file << ",3D,2D,profile,左x,左y,右x,右y,左x,左y,右x,右y\n";

        auto pcd_files = splitString(conf["file"]);
        auto model_files = splitString(conf["model"]);
        //string root = conf["root"];

        for (int i = 0; i < pcd_files.size(); i++)
        {
            cout << pcd_files[i] << endl;
            try
            {
                auto ret = save2D(root, pcd_files[i]);
                ret_total.push_back(ret);
                //save3D(root, pcd_files[i]);
            }
            catch (const char* errorMessage)
            {
                std::cerr << "Error: " << errorMessage << std::endl;
            }
            //save3D(root, pcd_files[i], filterNoiseDouble);
            //saveSkewness(root, pcd_files[i], model_files[i]);
            //if (model_files.size() > i)
            //    save(root, pcd_files[i], model_files[i]);
            //else
            //    save(root, pcd_files[i]);
        }

        //{     
        //    string root = conf["root"];
        //    save(root, "*1_X.pcd");
        //    save(root, "*2_Y.pcd");
        //    save(root, "*3_X.pcd");
        //    save(root, "*4_Y.pcd");
        //}
    }
    file.close();
    {
        vector<string> paths;
        vector<float> max_values;
        vector<float> min_values;

        for (auto& max_min : ret_total)
        {
            paths.push_back(max_min[1].first);
            max_values.push_back(max_min[0].second);
            min_values.push_back(max_min[1].second);
        }

        for (auto& val : paths)
        {
            file_sum << val << ",";
        }
        file_sum << "\n";
        for (auto& val : max_values)
        {
            file_sum << val << ",";
        }
        file_sum << "\n";
        for (auto& val : min_values)
        {
            file_sum << val << ",";
        }
        file_sum << "\n";
    }
}