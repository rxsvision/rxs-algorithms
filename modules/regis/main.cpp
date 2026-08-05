#include"Hole.h"
#include<vector>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include<pcl/common/transforms.h>
#include<pcl/common/pca.h>

//#define CZX_DEBUG

//#include"czxTool.h"

using namespace std;

auto conf = czxTool::readProfile("regis.czx");

//CP gatherCenter(string path, string hole_pos_path)
//{
//    CP ret(new CloudT);
//    CP cloud(new CloudT);
//    pcl::io::loadPCDFile(path, *cloud);
//
//    Hole ho;
//    vector<double> bound_points = arsenal::readDoubleFromFile(hole_pos_path);
//    CP h1(new CloudT);
//    {
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(cloud);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(bound_points[0], bound_points[1]);
//        pass.filter(*h1);
//        pass.setInputCloud(h1);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(bound_points[2], bound_points[3]);
//        pass.filter(*h1);
//    }
//    CP h2(new CloudT);
//    {
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(cloud);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(bound_points[0 + 4], bound_points[1 + 4]);
//        pass.filter(*h2);
//        pass.setInputCloud(h2);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(bound_points[2 + 4], bound_points[3 + 4]);
//        pass.filter(*h2);
//    }
//    CP h3(new CloudT);
//    {
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(cloud);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(bound_points[0 + 4 + 4], bound_points[1 + 4 + 4]);
//        pass.filter(*h3);
//        pass.setInputCloud(h3);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(bound_points[2 + 4 + 4], bound_points[3 + 4 + 4]);
//        pass.filter(*h3);
//    }
//    cloud = nullptr;
//
//    CP b1 = ho.KNNBoundExtract(h1);
//    Tool::showComparison(h1, b1);
//    {
//        CzxComparison asf(b1);
//
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(b1);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(bound_points[0] + 0.02, bound_points[1] - 0.02);
//        pass.filter(*b1);
//        pass.setInputCloud(b1);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(bound_points[2] + 0.02, bound_points[3] - 0.02);
//        pass.filter(*b1);
//    }
//    PointT c1 = ho.centerFit(b1);
//    ret->push_back(c1);
//    CP b2 = ho.KNNBoundExtract(h2);
//    Tool::showComparison(h2, b2);
//
//    {
//        CzxComparison asf(b2);
//
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(b2);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(bound_points[0 + 4] + 0.02, bound_points[1 + 4] - 0.02);
//        pass.filter(*b2);
//        pass.setInputCloud(b2);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(bound_points[2 + 4] + 0.02, bound_points[3 + 4] - 0.02);
//        pass.filter(*b2);
//    }
//    PointT c2 = ho.centerFit(b2);
//    ret->push_back(c2);
//    CP b3 = ho.KNNBoundExtract(h3);
//    Tool::showComparison(h3, b3);
//
//    {
//        CzxComparison asf(b3);
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(b3);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(bound_points[0 + 4 + 4] + 0.02, bound_points[1 + 4 + 4] - 0.02);
//        pass.filter(*b3);
//        pass.setInputCloud(b3);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(bound_points[2 + 4 + 4] + 0.02, bound_points[3 + 4 + 4] - 0.02);
//        pass.filter(*b3);
//
//    }
//    PointT c3 = ho.centerFit(b3);
//    ret->push_back(c3);
//    return ret;
//}

void main()
{
    //vector<string> path_list;
    //{
    //    string root = "D:\\download\\line\\";
    //    string inPath = root + string("*.pcd");
    //    __int64 handle;
    //    struct _finddata_t fileinfo;
    //    handle = _findfirst(inPath.c_str(), &fileinfo);
    //    if (handle == -1)
    //        return;

    //    do
    //    {
    //        //找到的文件的文件名
    //        string szPath;
    //        szPath = root + string(fileinfo.name);
    //        path_list.push_back(szPath);
    //    } while (!_findnext(handle, &fileinfo));
    //}

    //auto path_list = arsenal::pathGather(conf["root"], "*zx.pcd");
    //vector<CP> x_hole;
    //for (auto path : path_list)
    //{
    //    x_hole.push_back(gatherCenter("registration_p1.pcd", "x_Hole.txt"));
    //}

    //path_list = arsenal::pathGather(conf["root"], "*yf.pcd");
    //vector<CP> y_hole;
    //for (auto path : path_list)
    //{
    //    y_hole.push_back(gatherCenter("registration_p2.pcd", "y_Hole.txt"));
    //}

    //for (int i = 0; i < x_hole.size(); i++)
    //{
    //    Eigen::Matrix4f transformation;
    //    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    //    transformation_estimation.estimateRigidTransformation(*x_hole[i], *y_hole[i], transformation);
    //    //cout << transformation << endl;
    //    pcl::transformPointCloud(*x_hole[i], *x_hole[i], transformation);
    //    Eigen::Matrix3f diff = x_hole[i]->getMatrixXfMap(3, 4, 0) - y_hole[i]->getMatrixXfMap(3, 4, 0);
    //    Vector3f bias = diff.colwise().squaredNorm();
    //    cout << bias << endl;
    //}

    HMODULE hDLL = LoadLibraryW(L"czxToolkit.dll");
    typedef Eigen::Matrix4f(__cdecl* MYTYPE)(CP c1, CP c2, string hole_pos_path_x, string hole_pos_path_y);
    MYTYPE registration = (MYTYPE)GetProcAddress(hDLL, "registration");
    auto path_list1 = arsenal::pathGather(conf["root"], "*zx.pcd");
    auto path_list2 = arsenal::pathGather(conf["root"], "*yf.pcd");
    for (int i = 0; i < path_list1.size(); i++)
    {
        CP x(new CloudT);
        CP y(new CloudT);
        //pcl::io::loadPCDFile(path_list1[i], *x);
        //pcl::io::loadPCDFile(path_list2[i], *y);

        //auto transformation = registration(x, y, "zx.czx", "yf.czx");
        pcl::io::loadPCDFile("registration_p1.pcd", *x);
        pcl::io::loadPCDFile("registration_p2.pcd", *y);

        #ifdef CZX_DEBUG
        CP copy_x(new CloudT);
        CP copy_y(new CloudT);
        *copy_x = *x;
        *copy_y = *y;
        #endif

        CzxTimer sagdvgjd("registration");
        auto transformation = registration(x, y, "x_Hole.txt", "y_Hole.txt");

        pcl::transformPointCloud(*x, *x, transformation);
        Eigen::Matrix3f diff = x->getMatrixXfMap(3, 4, 0) - y->getMatrixXfMap(3, 4, 0);
        Vector3f bias = diff.colwise().squaredNorm();
        cout << bias << endl;

        #ifdef CZX_DEBUG
        pcl::transformPointCloud(*copy_x, *copy_x, transformation);
        #ifdef RXS_HAS_VISUALIZATION
        Tool::showComparison(copy_x, copy_y);
        #endif
        #endif
    }

}

//void main()
//{
//    //for lxz
//    CP cloud(new CloudT);
//    pcl::io::loadPCDFile("0_0.pcd", *cloud);
//
//   
//
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud(cloud);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(2.22, 90);
//    pass.filter(*cloud);
//
//    Hole ho;
//    CP b3 = ho.KNNBoundExtract(cloud);
//    Tool::showComparison(cloud, b3);
//
//    {
//        CzxComparison asf(b3);
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(b3);
//        pass.setFilterFieldName("x");
//        pass.setFilterLimits(75, 90);
//        pass.filter(*b3);
//        pass.setInputCloud(b3);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits(125, 132);
//        pass.filter(*b3);
//
//    }
//
//    pcl::PCA<PointT> pca;
//    pca.setInputCloud(b3);
//    Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
//    if (projection(2, 2) < 0)
//    {
//        projection = -projection;
//    }
//
//    b3->getMatrixXfMap(3, 4, 0) = projection * b3->getMatrixXfMap(3, 4, 0);
//
//    PointT c3 = ho.centerFit(b3);
//
//    c3.getVector3fMap() = projection.inverse() * c3.getVector3fMap();
//
//    cout << c3.getVector3fMap();
//}