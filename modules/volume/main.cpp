#include"czxTool.h"

/// <summary>
/// 
/// </summary>
/// <param name="cloud">输入待测体积的点云/// </param>
/// <param name="xyInterval">点之间xy的间隔，如果xy间隔不一致，可以取它们乘积的开方</param>
/// <returns>输出待测点云与xy平面围成的体积</returns>
__declspec(dllexport) double volume(CP cloud, double xyInterval)
{
    double sum = 0;
    for (auto p : *cloud)
    {
        sum += xyInterval * xyInterval * p.z;
    }
    return sum;
}


int main()
{
    double radius = 1.0;
    int resolution = 50;
    CP cloud(new CloudT);

    double xyInterval = 0.008;
    double thetaInterval = xyInterval / radius; // 计算theta的增量

    //生成球
    for (double x = -radius; x <= radius; x += xyInterval) {
        for (double y = -radius; y <= radius; y += xyInterval) {
            double z_squared = radius * radius - x * x - y * y;

            // Check if the point is within the sphere
            if (z_squared >= 0) {
                double z = sqrt(z_squared);

                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;

                cloud->points.push_back(point);
            }
        }
    }

    ////测试球体积测量    
    HMODULE hDLL = LoadLibraryW(L"czxToolkit.dll");
    typedef double(__cdecl* MYTYPE)(CP cloud, double xyInterval);
    MYTYPE volume_dll = (MYTYPE)GetProcAddress(hDLL, "volume");
    double sum = volume_dll(cloud, xyInterval);
    cout << sum;

    #ifdef RXS_HAS_VISUALIZATION
    Tool::show(cloud);
    #endif
	return 0;
}