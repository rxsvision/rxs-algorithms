#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/point_cloud.h>
#include "czxTool.h"
#include"GA.h"
#include"PSO.h"
//#include"TheadHull.h"
#include"MultipleDllConvex.h"

#define CZX_DEBUG

#ifdef CZX_DEBUG
Eigen::Vector3f norm_debug;
#endif // CZX_DEBUG



pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    //CzxTimer dgfsg(__func__);

    // 创建凸包对象
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);
    // 计算凸包
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*hull_cloud);

    return hull_cloud;


    //ThreadHull th(100, cloud);
    //th.process();

    //hull.setInputCloud(th.hull_points);
    //hull.setDimension(3);
    //// 计算凸包
    //hull.reconstruct(*hull_cloud);

    //pcl::io::savePCDFileBinary("myhull.pcd", *hull_cloud);
    //return hull_cloud;

    //return th.hull_points;
        
    // 获取当前线程的唯一标识符
    std::thread::id threadId = std::this_thread::get_id();
    // 创建 DLL 文件名，以 threadId 命名
    std::string dllFileName = "./czxToolkit/czxToolkit_" + to_string((unsigned int)&threadId) + ".dll";

    arsenal::copyFile_czx("czxToolkit.dll", dllFileName);

    HMODULE hDLL = LoadLibraryW(arsenal::ConvertToLPCWSTR(dllFileName));
    cout << hDLL << endl;
    typedef CP(__cdecl* CC)(CP cloud);
    CC cch = (CC)GetProcAddress(hDLL, "computeConvexHull");
    return cch(cloud);
}


CP computeConvexHullMultiple(CP cloud)
{
    CzxTimer sdgadgea(__func__);
    //vector<CP> sub_clouds;
    ////int sub_clouds_size = std::thread::hardware_concurrency()-1;
    //int sub_clouds_size = 20;

    //size_t cloud_size = cloud->points.size();
    //size_t chunk_size = cloud_size / sub_clouds_size;

    //auto matrix = cloud->getMatrixXfMap(3, 4, 0);
    //for (size_t i = 0; i < sub_clouds_size-1; ++i)
    //{
    //    CP tmp(new CloudT);
    //    int start = i * chunk_size;
    //    Eigen::MatrixXf sub_matrix = matrix.block(0, start, matrix.rows(), chunk_size);
    //    tmp->resize(chunk_size);
    //    tmp->getMatrixXfMap(3, 4, 0) = sub_matrix;
    //    sub_clouds.push_back(tmp);
    //}
    //{
    //    Eigen::MatrixXf sub_matrix = matrix.block(0, (sub_clouds_size - 1) * chunk_size, matrix.rows(), cloud_size - (sub_clouds_size - 1) * chunk_size);
    //    CP tmp(new CloudT);
    //    tmp->resize(cloud_size - (sub_clouds_size - 1) * chunk_size);
    //    tmp->getMatrixXfMap(3, 4, 0) = sub_matrix;
    //    sub_clouds.push_back(tmp);
    //}
    //
    //vector<CP> sub_hull;

    //#pragma omp parallel for
    //for (int i = 0; i < sub_clouds.size(); i++)
    //{
    //    CP hull = computeConvexHull(sub_clouds[i]);
    //    #pragma omp critical
    //    {
    //        sub_hull.push_back(hull);
    //    }
    //}
    //CP ret(new CloudT);
    //for (int i = 0; i < sub_hull.size(); i++)
    //{
    //    *ret += *sub_hull[i];
    //}
    //ret = computeConvexHull(ret);
    //return ret;

    MultipleDllConvex mdc(20);
    mdc.setInputCloud(cloud);
    auto ret = mdc.process();
    return ret;
}

Eigen::Vector3f getPlaneNorm(PointT a, PointT b, PointT c)
{
    auto x = b.getVector3fMap() - a.getVector3fMap();
    auto y = c.getVector3fMap() - a.getVector3fMap();
    auto z = x.cross(y);
    z.normalize();
    return z;
}

#ifdef CZX_DEBUG
PointT best_a, best_b, best_c;
#endif

double MZFlatness(CP cloud)
{
    CzxTimer aga(__func__);
    double min_flatness = 9999;
    CP hull = computeConvexHull(cloud);
    int n = hull->size();
    double max_dif_z_of_plane = 0.4;
    double max_dif_x_of_plane = 2;

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            //if (abs(hull->points[i].z - hull->points[j].z) > max_dif_z_of_plane)
            //    continue;
            //if (abs(hull->points[i].x - hull->points[j].x) > max_dif_x_of_plane)
            //    continue;
            for (int k = j + 1; k < n; ++k) {
                //if (hull->points[k].z - hull->points[i].z > max_dif_z_of_plane)
                //    continue;
                //if (hull->points[k].z - hull->points[j].z > max_dif_z_of_plane)
                //    continue;
                auto norm = getPlaneNorm(hull->points[i], hull->points[j], hull->points[k]);                
                //auto z_hull = norm.transpose() * cloud->getMatrixXfMap(3, 4, 0);
                auto z_hull = norm.transpose() * hull->getMatrixXfMap(3, 4, 0);
                double flatness = z_hull.maxCoeff() - z_hull.minCoeff();                
                if (flatness < min_flatness)
                {
                    min_flatness = flatness;
                    #ifdef CZX_DEBUG
                    norm_debug = norm;
                    best_a = hull->points[i];
                    best_b = hull->points[j];
                    best_c = hull->points[k];
                    #endif
                }
            }
        }
    }
    return min_flatness;
}

double MLSFlatness(CP cloud)
{
    CzxTimer aga(__func__);
    
    MatrixXf cloud_eg = cloud->getMatrixXfMap(3, 4, 0);
    if (cloud_eg.cols() > 1000000)
        cloud_eg.conservativeResize(cloud_eg.rows(), 100);
    auto mean = cloud_eg.rowwise().mean();
    cloud_eg = cloud_eg.colwise() - cloud_eg.rowwise().mean();

    float x2 = cloud_eg.row(0).squaredNorm();
    float xy = cloud_eg.row(0) * cloud_eg.row(1).transpose();
    float x = cloud_eg.row(0).sum();

    float y2 = cloud_eg.row(1).squaredNorm();
    float y = cloud_eg.row(1).sum();
    int n = cloud->size();

    float xz = cloud_eg.row(0) * cloud_eg.row(2).transpose();
    float yz = cloud_eg.row(1) * cloud_eg.row(2).transpose();
    float z = cloud_eg.row(2).sum();

    Matrix3f A;
    A << x2, xy, x,
        xy, y2, y,
        x, y, n;

    Vector3f b;
    b << xz, yz, z;
    Vector3f X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    
    //Vector3f diff = b - A * X;
    //cout << diff << endl;
    //if (diff.norm() > 100)
    //{
    //    auto X_delta = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(diff);
    //    cout << X_delta << endl;
    //    X += X_delta;
    //}

    //diff = A * X - b;
    //cout << diff << endl;

    //Vector3f X = A.lu().solve(b);
    Vector3f norm(X[0], X[1], -1);
    norm.normalize();

    auto z_data = norm.transpose() * cloud->getMatrixXfMap(3,4,0);
    //int max_index, min_index;
    //double flatness = z_data.maxCoeff(&max_index) - z_data.minCoeff(&min_index);
    double flatness = z_data.maxCoeff() - z_data.minCoeff();


    #ifdef CZX_DEBUG
    norm_debug = norm;
    #endif
    return flatness;
}

double GAFlatness(CP cloud)
{
    CzxTimer aga(__func__);
    CP hull = computeConvexHull(cloud);
    GA ga(hull);
    ga.process();
    return ga.getBest();
}

double PSOFlatness(CP cloud, Vector3f &norm_out)
{
    CzxTimer aga(__func__);
    CP hull = computeConvexHullMultiple(cloud);
    //CP hull = computeConvexHull(cloud);
    PSO pso(hull);
    pso.process();

    auto norm = pso.getBestPos();
    norm.normalize();
    norm_out = norm;
    VectorXf z_data = norm.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    int min_index, max_index;
    double flatness = z_data.maxCoeff(&max_index) - z_data.minCoeff(&min_index);
    #ifdef CZX_DEBUG
    cout << "pso global:" << flatness << endl;
    #endif
    double best_fitness = pso.getBest();
    return best_fitness;
}

double PSOFlatness(CP cloud)
{
    CzxTimer aga(__func__);
    CP hull = computeConvexHullMultiple(cloud);
    //CP hull = computeConvexHull(cloud);
    PSO pso(hull);
    pso.process();

    //auto norm = pso.getBestPos();
    //norm.normalize();
    //VectorXf z_data = norm.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    //int min_index, max_index;
    //double flatness = z_data.maxCoeff(&max_index) - z_data.minCoeff(&min_index);
    //#ifdef CZX_DEBUG
    //cout << "pso global:" << flatness << endl;
    //#endif
    return pso.getBest();
}

int main()
{
    auto conf = czxTool::readProfile("mz_conf.czx");
    CP cloud(new CloudT);
    pcl::io::loadPCDFile(conf["path"], *cloud);

    std::ofstream file("report.csv", std::ios::app);
    vector<int> times_pso, times_mls;
    vector<double> flatnesses_pso, flatnesses_mls;
    for (int i=6500;i>=1000;i-=500)
    {
        cout << "点数目" << i << "万" << endl;
        cloud->resize(i * 10000);
        auto start = std::chrono::system_clock::now();
        flatnesses_mls.push_back(MLSFlatness(cloud));
        auto ends = std::chrono::system_clock::now();
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(ends - start);
        times_mls.push_back(duration.count());
        
        
        start = std::chrono::system_clock::now();
        flatnesses_pso.push_back(PSOFlatness(cloud));
        ends = std::chrono::system_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(ends - start);
        times_pso.push_back(duration.count());
    }
    file << "Number of Points," << "PSO," << "MLS" << "\n";
    for (int i = 6500, index=0; i >= 1000; i -= 500, index++)
    {
        file << i << "," << "Time: " << times_pso[index] << "  Flatness: " << flatnesses_pso[index] << ",";
        file << "Time: " << times_mls[index] << "  Flatness: " << flatnesses_mls[index] << "\n";
    }
    
    
    return 0;
    double base_line=0;

    base_line = MLSFlatness(cloud);
    cout << "mls :" << base_line << endl;

    int test_num = stoi(conf["test_num"]);

    //int bad_ga = 0;
    //for (int _ = 0; _ < test_num; _++)
    //{
    //    //cout << "ga :" << GAFlatness(cloud) << endl;
    //    auto ga_ret = GAFlatness(cloud);
    //    if (ga_ret > base_line)
    //        bad_ga++;
    //}
    //cout << "bad ga: " << bad_ga << endl;


    //base_line = 0.563;
    int bad_pso = 0;
    double max_pso = -1, min_pso = 99999;
    for (int _ = 0; _ < test_num; _++)
    {
        Vector3f norm;
        auto pso_ret = PSOFlatness(cloud, norm);
        cout << "pso :" << pso_ret << endl;
        if (pso_ret > base_line)
            bad_pso++;
        
        max_pso = max(max_pso, pso_ret);
        min_pso = min(min_pso, pso_ret);


        if (conf["visual"] == "1")
        {
            CP copy_cloud(new CloudT);
            *copy_cloud = *cloud;
            copy_cloud->getMatrixXfMap(3, 4, 0) = arsenal::constructRotationFromZ(norm) * copy_cloud->getMatrixXfMap(3, 4, 0);
            CP high_low(new CloudT);
            int min_index, max_index;
            double flatness = copy_cloud->getMatrixXfMap(3, 4, 0).row(2).maxCoeff(&max_index) - copy_cloud->getMatrixXfMap(3, 4, 0).row(2).minCoeff(&min_index);
            cout << flatness - pso_ret << endl;
            high_low->push_back(copy_cloud->points[max_index]);
            high_low->push_back(copy_cloud->points[min_index]);
            pcl::io::savePCDFileBinary("pso_cloud.pcd", *copy_cloud);
            pcl::io::savePCDFileBinary("pso_hl.pcd", *high_low);
            //Tool::showComparison(copy_cloud, high_low, 2, 5);
        }
    }
    cout << "max pso: " << max_pso << endl;
    cout << "min pso: " << min_pso << endl;
    cout << "bad pso: " << bad_pso << endl;


    cout << "mz :" << MZFlatness(cloud) << endl;
    #ifdef CZX_DEBUG
    VectorXf z_data = norm_debug.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    int min_index, max_index;
    double flatness = z_data.maxCoeff(&max_index) - z_data.minCoeff(&min_index);
    cout << "mz global:" << flatness << endl;
    #endif

    //#ifdef CZX_DEBUG
    //CP max_min(new CloudT);
    //max_min->push_back(cloud->points[max_index]);
    //max_min->push_back(cloud->points[min_index]);
    //CP hull = computeConvexHull(cloud);


    //auto rotation = arsenal::constructRotationFromZ(norm_debug);

    //pcl::io::savePCDFileBinary("hull_before.pcd", *hull);
    //cloud->getMatrixXfMap(3, 4, 0) = rotation * cloud->getMatrixXfMap(3, 4, 0);
    //max_min->getMatrixXfMap(3, 4, 0) = rotation * max_min->getMatrixXfMap(3, 4, 0);
    //hull->getMatrixXfMap(3, 4, 0) = rotation * hull->getMatrixXfMap(3, 4, 0);

    ////flatness = cloud->getMatrixXfMap(3, 4, 0).row(2).maxCoeff(&max_index) - cloud->getMatrixXfMap(3, 4, 0).row(2).minCoeff(&min_index);
    ////cout << flatness << endl;

    ////cout << max_index << endl;
    ////cout << min_index << endl;

    //CP best_plane(new CloudT);

    //best_plane->push_back(best_a);
    //best_plane->push_back(best_b);
    //best_plane->push_back(best_c);
    //pcl::io::savePCDFileBinary("best_plane_before.pcd", *best_plane);
    //best_plane->getMatrixXfMap(3, 4, 0) = rotation * best_plane->getMatrixXfMap(3, 4, 0);

    //pcl::io::savePCDFileBinary("best_plane.pcd", *best_plane);
    //pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
    //pcl::io::savePCDFileBinary("maxmin.pcd", *max_min);
    //pcl::io::savePCDFileBinary("hull.pcd", *hull);
    //#endif // CZX_DEBUG

    //hull = computeConvexHull(cloud);
    //pcl::io::savePCDFileBinary("hull2.pcd", *hull);


    //Tool::showComparison(hull, max_min);
    //cout << flatness << endl;
}