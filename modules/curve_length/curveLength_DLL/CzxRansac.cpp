#include"CzxRansac.h"
#include <pcl/filters/random_sample.h>

#include <random> 

#include<string.h>

#include <pcl/filters/project_inliers.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。

#include "czxTool.h"



using namespace std;

Ransac::Ransac() :coff(5)
{
    config = czxTool::readProfile("ransac.czx");

    threshold_inlier = stof(config["threshold_inlier"]);
    sample_num = stof(config["sample_num"]);
    max_iter = stof(config["max_iter"]);
    approximate_inlier = stof(config["approximate_inlier"]);

    approximate_sample.reset(new CloudT);
}

CP
Ransac::randSample(CP cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample(new pcl::PointCloud<pcl::PointXYZ>);

    //sample_index = vector<int>();
    int real_num = 0;
    for (int i = 0; i < sample_num; i++,real_num++)
    {
        // 初始化随机数生成器
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(1, cloud->size());

        // 生成1到x之间的随机整数
        int randomNum = dis(gen);
        if (valid_num[randomNum] < 0 && real_num<cloud->size())
        {
            i--;
            continue;
        }
        sample->push_back(cloud->points[randomNum]);
        //sample_index.push_back(randomNum);
    }


    //pcl::RandomSample<pcl::PointXYZ> sampler;
    //sampler.setInputCloud(cloud);
    //sampler.setSample(sample_num);
    //sampler.filter(*sample);
    return sample;
}


void
Ransac::fitOnce(CP sample)
{
    //CzxTimer fbhgdjusfgvuw(__func__);
    int N = sample->points.size();
    Eigen::MatrixXf cloudMatrix(2, N);

    for (size_t i = 0; i < N; ++i) {
        cloudMatrix(0, i) = sample->points[i].x;
        cloudMatrix(1, i) = sample->points[i].y;
    }


    {
        #define CX cloudMatrix.row(0).array()
        #define CY cloudMatrix.row(1).array()
        #define XY(a, b) (CX.pow(a) * CY.pow(b)).sum()
        #define Y(a) (CY.pow(a)).sum()
        #define X(a) (CX.pow(a)).sum()


        Eigen::MatrixXf A = (Eigen::MatrixXf(5, 5) << XY(2, 2), XY(1, 3), XY(2, 1), XY(1, 2), XY(1, 1),
            XY(1, 3), Y(4), XY(1, 2), Y(3), Y(2),
            XY(2, 1), XY(1, 2), X(2), XY(1, 1), X(1),
            XY(1, 2), Y(3), XY(1, 1), Y(2), Y(1),
            XY(1, 1), Y(2), X(1), Y(1), N).finished();

        //cout << "A:" << endl;
        //czxTool::printMatrixXf(A);

        Eigen::VectorXf B = (Eigen::VectorXf(5) <<
            XY(3, 1), XY(2, 2), X(3), XY(2, 1), X(2)
            ).finished();

        //cout << "B:" << endl;
        //czxTool::printMatrixXf(B);

        //Eigen::MatrixXf AQr = A.colPivHouseholderQr().matrixQR();

        //cout << "AQe:" << endl;
        //czxTool::printMatrixXf(AQr);

        coff = A.colPivHouseholderQr().solve(-B);

        //for (int i = 0; i < coff.size(); ++i) {
        //    std::cout << "coff[" << i << "] = " << coff(i) << "\t";
        //}
        //cout << endl << endl;
    }


    {
        #define A *coff[0]
        #define B *coff[1]
        #define C *coff[2]
        #define D *coff[3]
        #define E *coff[4]

        ellipse_x = (2 B C - 1 A D) / (1 A A - 4 B);
        ellipse_y = (2 D - 1 A C) / (1 A A - 4 B);

#define UP (2 * (1 A C D - 1 B C C - 1 D D + 4 B E - 1 A A E))

        ellipse_a = sqrt(
            UP
            / ((1 A A - 4 B) * (1 B + 1 - sqrt(1 A A + (1 - 1 B) * (1 - 1 B))))
        );
        ellipse_b = sqrt(
            UP /
            ((1 A A - 4 B) * (1 B + 1 + sqrt(1 A A + (1 - 1 B) * (1 - 1 B))))
        );

        ellipse_c = sqrt(ellipse_a * ellipse_a - ellipse_b * ellipse_b);


        ellipse_theta = atan(sqrt(
            (ellipse_a * ellipse_a - ellipse_b * ellipse_b B) /
            (ellipse_a * ellipse_a B - ellipse_b * ellipse_b)
        ));

        #undef A
        #undef B
        #undef C
        #undef D
        #undef E
    }

    {
        if (coff[0] <= 0)
        {
            f1[0] = ellipse_x + ellipse_c * cos(ellipse_theta);
            f1[1] = ellipse_y + ellipse_c * sin(ellipse_theta);

            f2[0] = ellipse_x - ellipse_c * cos(ellipse_theta);
            f2[1] = ellipse_y - ellipse_c * sin(ellipse_theta);
        }
        else
        {
            f1[0] = ellipse_x + ellipse_c * cos(ellipse_theta);
            f1[1] = ellipse_y - ellipse_c * sin(ellipse_theta);

            f2[0] = ellipse_x - ellipse_c * cos(ellipse_theta);
            f2[1] = ellipse_y + ellipse_c * sin(ellipse_theta);
        }
    }
}

float
Ransac::p2ellipse(PointT p)
{
    //y = mx + b
    float m = (p.y - ellipse_y) / (p.x - ellipse_x);
    float b = ellipse_y - m * ellipse_x;


    //直线与椭圆交点
    //Ax^2+Bx+C=0
    float A = 1 + coff[0] * m + coff[1] * m * m;
    float B = coff[2] + coff[0] * b + 2 * m * b * coff[1] + m * coff[3];
    float C = coff[4] + coff[1] * b * b + b * coff[3];


    //理论上此处不可能小于0
    float discriminant = B * B - 4 * A * C;

    float x1 = (-B + sqrt(discriminant)) / (2 * A);
    float x2 = (-B - sqrt(discriminant)) / (2 * A);

    float y1 = m * x1 + b;
    float y2 = m * x2 + b;

#define DISTANCE(X, Y) sqrt((p.x-X)* (p.x - X)+(p.y-Y)* (p.y - Y))
    return min(DISTANCE(x1, y1), DISTANCE(x2, y2));
}

float
Ransac::p2focus(PointT p)
{
#define DISTANCE(X, Y) sqrt((p.x-X)* (p.x - X)+(p.y-Y)* (p.y - Y))

    return DISTANCE(f1[0], f1[1]) + DISTANCE(f2[0], f2[1]);
}

state
Ransac::evaluate(CP cloud)
{
    bool illegal = (abs(1 - (ellipse_a / ellipse_b)) > 0.15);
    //if (illegal)
    //{
    //    for (int index : sample_index)
    //    {
    //        valid_num[index] -= 10;
    //    }
    //}

    ellipse_found.reset(new CloudT);
    approximate_sample.reset(new CloudT);
    int num = 0;
    int i = 0;
    for (auto p : cloud->points)
    {
        //float bias = p2ellipse(p);
        //num += int(p2ellipse(p) < threshold_inlier);

        //float dif = abs(p2focus(p) - 2 * ellipse_a);
        float bias = abs(p2focus(p) - 2 * ellipse_a);
        if (bias < threshold_inlier)
        {
            num++;
            ellipse_found->push_back(p);
            if (illegal)
            {
                valid_num[i]--;
                valid_sum--;
            }
            else
            {
                valid_num[i]++;
                valid_sum++;
            }
            //if (!illegal)
            //{
            //    valid_num[i]++;
            //    valid_sum++;
            //}
        }
        if (bias < approximate_inlier * threshold_inlier)
        {
            approximate_sample->push_back(p);
        }
        i++;
    }

    if(illegal) return state::Fail;

    if (num > cloud->size() * stof(config["done_ratio"]))
        return state::Done;
    else if (num > cloud->size() * stof(config["success_ratio"]))
        return state::Success;
    else
        return state::Fail;
}

//Eigen::Vector3f 
//Ransac::minProjectPlane(CP cloud)
//{
//    Eigen::MatrixXf eigen_cloud = cloud->getMatrixXfMap(3, 4, 0);
//
//    float x = eigen_cloud.row(0).array().mean();
//    float y = eigen_cloud.row(1).array().mean();
//    float z = eigen_cloud.row(2).array().mean();
//
//    float x2 = eigen_cloud.row(0).array().pow(2).mean();
//    float y2 = eigen_cloud.row(1).array().pow(2).mean();
//    float z2 = eigen_cloud.row(2).array().pow(2).mean();
//
//
//    Eigen::MatrixXf A = (Eigen::MatrixXf(4, 3) << 
//        x2*x2, y*y, z*z,
//        x*x, y2*y2, z*z,
//        x*x, y*y, z2*z2,
//        1, 1, 1
//        ).finished();
//
//    Eigen::Vector4f B(0,0,0, 1);
//
//
//    Eigen::MatrixXf AQr = A.colPivHouseholderQr().matrixQR();
//
//    cout << "AQe:" << endl;
//    czxTool::printMatrixXf(AQr);
//
//    Eigen::Vector3f coff = A.colPivHouseholderQr().solve(-B);
//    coff = coff.array().sqrt();
//    return coff;
//}

//CP 
//Ransac::projectPlane(CP cloud)
//{
//    Eigen::Vector3f coff = minProjectPlane(cloud);
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//
//    // 手动填充平面参数
//    coefficients->values.resize(4);
//    coefficients->values[0] = coff[0];  // 法线向量的X分量
//    coefficients->values[1] = coff[1];  // 法线向量的Y分量
//    coefficients->values[2] = coff[2];  // 法线向量的Z分量
//    coefficients->values[3] = 0;  // 距离参数
//
//    
//    CP cloud_projected(new CloudT);
//    pcl::ProjectInliers<pcl::PointXYZ> proj;
//    proj.setModelType(pcl::SACMODEL_PLANE);
//    proj.setInputCloud(cloud);
//    proj.setModelCoefficients(coefficients);
//    proj.filter(*cloud_projected);
//    return cloud_projected;
//}

mutex mtx;

bool
Ransac::fit(CP cloud)
{
    CzxTimer fbhgdjusfgvuw(__func__);

    valid_num.resize(cloud->size(), 0);
    valid_sum = 0;
    int iter = 0;
    bool success = true;
    //while(true)
    //{
    //    CP sample;
    //    if (approximate_sample->size() > 20)
    //    {
    //        sample = approximate_sample;
    //    }
    //    else
    //    {
    //        sample = randSample(cloud);
    //    }
    //    fitOnce(sample);
    //    iter++;
    //    if (iter > max_iter) {
    //        success = false;
    //        break;
    //    }

    //    state ret = evaluate(cloud);
    //    if (ret == state::Success || ret== state::Done)
    //    {
    //        success = true;
    //        break;
    //    }
    //}
    state ret;
    for (int i = 0; i < max_iter; i++)
    {
        fitOnce(randSample(cloud));
        ret = evaluate(cloud);
        if (ret == state::Done)
        {
            cout << "DONE" << endl;
            break;

        }
    }

    CP valid_sample(new CloudT);

    if (ret != state::Done)
    {
        int valid = max_iter * stof(config["valid_ratio"]);
        while (valid_sample->size() < 10)
        {
            valid_sample.reset(new CloudT);
            for (int i = 0; i < valid_num.size(); i++)
            {
                if (valid_num[i] > valid)
                {
                    valid_sample->push_back(cloud->points[i]);
                }
            }
            valid *= 0.9;
            if (valid == 0) break;
        }
        fitOnce(valid_sample);

        ret = evaluate(cloud);
    }

    if (ret == state::Success || ret == state::Done) success = true;
    else success = false;
    

    //pcl::io::savePCDFile("cloud.pcd", *cloud);
    mtx.lock();
    //if (success)
    //    cout << "Success" << endl;
    //else
    //    cout << "Failure" << endl;
    if (!success)
        cout << "Failure" << endl;


    ////TODO 此处存疑，两幅点云z都为0，但是显示时不在一个平面上
    //{
    //    Eigen::MatrixXf tmp = cloud_copy->getMatrixXfMap(3, 4, 0);
    //    tmp = tmp.block<3, 10>(0, 0);
    //    cout << tmp << endl;
    //    tmp = fitted_ellipse->getMatrixXfMap(3, 4, 0);
    //    tmp = tmp.block<3, 10>(0, 0);
    //    cout << tmp << endl << endl;
    //}
    //Tool::showComparison(cloud_copy, fitted_ellipse, 5, 5);

    //for (int i = 0; i < fitted_ellipse->size(); i++)
    //{
    //    swap(fitted_ellipse->points[i].y, fitted_ellipse->points[i].z);
    //}

    //{
    //    Eigen::MatrixXf tmp = cloud_copy->getMatrixXfMap(3, 4, 0);
    //    tmp = tmp.block<3, 10>(0, 0);
    //    cout << tmp << endl;
    //    tmp = fitted_ellipse->getMatrixXfMap(3, 4, 0);
    //    tmp = tmp.block<3, 10>(0, 0);
    //    cout << tmp << endl << endl;
    //}
    //Tool::showComparison(cloud, valid_sample, 5, 5);
    CP cloud_copy(new CloudT);
    pcl::copyPointCloud(*cloud, *cloud_copy);
    cloud_copy->getMatrixXfMap(3, 4, 0).row(2).setZero();
    CP fitted_ellipse = drawEllipse();
    //Tool::showComparison(cloud_copy, fitted_ellipse, 5, 5);
    //if(valid_sample->size()>10)
    //    Tool::showComparison(cloud_copy, valid_sample, 5, 5);

    //Tool::showComparison(cloud, fitted_ellipse, 5, 5);
    //Tool::showComparison(cloud, ellipse_found, 5, 5);
    mtx.unlock();

    return success;
}


CP 
Ransac::drawEllipse()
{
    CP cloud(new CloudT);
    // 生成椭圆上的点云
    int numPoints = 1000; // 生成的点的数量
    for (int i = 0; i < numPoints; ++i) {
        float angle = 2.0 * M_PI * i / numPoints; // 角度从0到2π
        float x = ellipse_a * cos(angle);
        float y = ellipse_b * sin(angle);

        // 将点添加到点云中
        pcl::PointXYZ point(x, y, 0);
        cloud->push_back(point);
    }

    // 创建绕X轴旋转的旋转矩阵
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix = Eigen::AngleAxisf(ellipse_theta, Eigen::Vector3f::UnitZ());

    cloud->getMatrixXfMap(3, 4, 0) = rotationMatrix * cloud->getMatrixXfMap(3, 4, 0);
    Eigen::Vector3f vector(ellipse_x, ellipse_y, 0);
    cloud->getMatrixXfMap(3, 4, 0) += vector.replicate(1, cloud->size());
    return cloud;
}