#include "MultipleDllConvex.h"

MultipleDllConvex::MultipleDllConvex(int thread_num_in)
{
	thread_num = thread_num_in;

	for (int i = 0; i < thread_num; i++)
	{
		std::string dllFileName = "./czxToolkit/czxToolkit_" + to_string(i) + ".dll";
		arsenal::copyFile_czx("czxToolkit.dll", dllFileName);
	}
       
    for (int i = 0; i < thread_num; i++)
    {
        std::string dllFileName = "./czxToolkit/czxToolkit_" + to_string(i) + ".dll";
        HMODULE hDLL = LoadLibrary(arsenal::ConvertToLPCWSTR(dllFileName));
        if (hDLL)
        {
            loaded_dll.push_back(hDLL);
            CC cch = (CC)GetProcAddress(hDLL, "computeConvexHull");
            func_list.push_back(cch);
        }
        else
        {
            thread_num++;
        }
    }
}

MultipleDllConvex::~MultipleDllConvex()
{
    func_list.clear();
    //for (auto handle : loaded_dll)
    //{
    //    FreeLibrary(handle);
    //}
}

void MultipleDllConvex::setInputCloud(CP cloud_in)
{
	cloud = cloud_in;
}

CP MultipleDllConvex::solveConvexHull(CP clo, int id)
{    
    //CP copy(new CloudT);
    //*copy = *clo;
    CP ret = func_list[id](clo);

    return ret;
}

vector<CP> MultipleDllConvex::devideCloud()
{
    vector<CP> sub_clouds;
    int sub_clouds_size = thread_num;

    size_t cloud_size = cloud->points.size();
    size_t chunk_size = cloud_size / sub_clouds_size;

    auto matrix = cloud->getMatrixXfMap(3, 4, 0);
    for (size_t i = 0; i < sub_clouds_size - 1; ++i)
    {
        CP tmp(new CloudT);
        int start = i * chunk_size;
        Eigen::MatrixXf sub_matrix = matrix.block(0, start, matrix.rows(), chunk_size);
        tmp->resize(chunk_size);
        tmp->getMatrixXfMap(3, 4, 0) = sub_matrix;
        sub_clouds.push_back(tmp);
    }
    {
        Eigen::MatrixXf sub_matrix = matrix.block(0, (sub_clouds_size - 1) * chunk_size, matrix.rows(), cloud_size - (sub_clouds_size - 1) * chunk_size);
        CP tmp(new CloudT);
        tmp->resize(cloud_size - (sub_clouds_size - 1) * chunk_size);
        tmp->getMatrixXfMap(3, 4, 0) = sub_matrix;
        sub_clouds.push_back(tmp);
    }

    return sub_clouds;
}

CP MultipleDllConvex::process()
{
    auto sub_clouds = devideCloud();

    vector<CP> sub_hull;

    omp_set_num_threads(thread_num);
    #pragma omp parallel for
    for (int i = 0; i < sub_clouds.size(); i++)
    {
        CP hull = solveConvexHull(sub_clouds[i], i);
        #pragma omp critical
        {
            sub_hull.push_back(hull);
        }
    }
    CP ret(new CloudT);
    for (int i = 0; i < sub_hull.size(); i++)
    {
        *ret += *sub_hull[i];
    }
    ret = solveConvexHull(ret, 0);
    return ret;
}
