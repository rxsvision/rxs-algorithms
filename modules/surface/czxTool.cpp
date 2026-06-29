#include"czxTool.h"
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/filters/conditional_removal.h>
#include <unsupported/Eigen/FFT>


string CzxTimer::path = "";
std::mutex Tool::mtx_showComparison_ccss;

int id = 0;
void
selectCatch(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	CloudAndView unit = *(CloudAndView*)args;
	CloudT::Ptr cloud_ = unit.c;
	pcl::visualization::PCLVisualizer& viewer = *unit.v;
	//pcl::Indices ind;
	pcl::PointIndices p_ind;
	if (event.getPointsIndices(p_ind.indices) == -1)
		return;
	if (p_ind.indices.size() == 0)
		return;


	CloudT::Ptr selected_cloud(new CloudT);
	pcl::ExtractIndices<PointT> extract;
	extract.setIndices(boost::make_shared<pcl::PointIndices>(p_ind));
	extract.setInputCloud(cloud_);
	extract.filter(*selected_cloud);

	pcl::io::savePCDFile("selected.pcd", *selected_cloud);

	stringstream ss;
	ss << id;
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(selected_cloud, 255, 0, 0);
	viewer.addPointCloud(selected_cloud, red, ss.str());
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
	id++;
}

void
Tool::selectWindow(string file_name)
{
	CloudT::Ptr cloud(new CloudT);
	pcl::io::loadPCDFile(file_name, *cloud);
	pcl::visualization::PCLVisualizer v;
	viewer = &v;
	viewer->addPointCloud(cloud);
	CloudAndView unit;
	unit.v = viewer;
	unit.c = cloud;
	viewer->registerAreaPickingCallback(selectCatch, (void*)&unit);

	viewer->spin();
}

mutex tex;
void
Tool::show(CloudT::Ptr clo)
{
	tex.lock();
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(clo);
	viewer.spin();
	tex.unlock();

}

void
Tool::show(CloudNT::Ptr clo1)
{
	pcl::visualization::PCLVisualizer viewer;
	CloudT::Ptr clo(new CloudT);
	pcl::copyPointCloud(*clo1, *clo);
	viewer.addPointCloud(clo);
	viewer.spin();
}

void
Tool::showComparison(CloudT::Ptr c1, CloudT::Ptr c2)
{
	mtx_showComparison_ccss.lock();
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(c1, "1");
	viewer.addPointCloud(c2, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
	viewer.spin();
	mtx_showComparison_ccss.unlock();

}

void
Tool::showComparison(CloudT::Ptr c1, CloudT::Ptr c2, int size1, int size2, function<void(const pcl::visualization::KeyboardEvent&)> callback, string name)
{
	mtx_showComparison_ccss.lock();
	pcl::visualization::PCLVisualizer viewer;
	viewer.setWindowName(name);
	viewer.addPointCloud(c1, "1");
	viewer.addPointCloud(c2, "2");
	if (callback != nullptr) {
		// 注册键盘事件回调函数
		viewer.registerKeyboardCallback(callback);
	}
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size1, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size2, "2");
	viewer.spin();
	mtx_showComparison_ccss.unlock();
}

void Tool::showComparison(CloudT::Ptr c1, PointT p2, int size1, int size2, function<void(const pcl::visualization::KeyboardEvent&)> callback, string name)
{
	CP c2(new CloudT);
	c2->push_back(p2);
	mtx_showComparison_ccss.lock();
	pcl::visualization::PCLVisualizer viewer;
	viewer.setWindowName(name);
	viewer.addPointCloud(c1, "1");
	viewer.addPointCloud(c2, "2");
	if (callback != nullptr) {
		// 注册键盘事件回调函数
		viewer.registerKeyboardCallback(callback);
	}
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size1, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size2, "2");
	viewer.spin();
	mtx_showComparison_ccss.unlock();
}


void Tool::showComparison(CloudT::Ptr c1, CloudT::Ptr c2, CloudT::Ptr c3, int size1, int size2, int size3, function<void(const pcl::visualization::KeyboardEvent&)> callback, string name)
{
	mtx_showComparison_ccss.lock();
	pcl::visualization::PCLVisualizer viewer;
	viewer.setWindowName(name);
	viewer.addPointCloud(c1, "1");
	viewer.addPointCloud(c2, "2");
	viewer.addPointCloud(c3, "3");
	if (callback != nullptr) {
		// 注册键盘事件回调函数
		viewer.registerKeyboardCallback(callback);
	}
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size1, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size2, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "3");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size3, "3");
	viewer.spin();

	mtx_showComparison_ccss.unlock();
}

void
Tool::showComparison(CloudT::Ptr c1, CloudT::Ptr c2, bool coor)
{
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.5, 0.5, 0.5);
	viewer.addCoordinateSystem(1.0, "3");
	viewer.addPointCloud(c1, "1");
	viewer.addPointCloud(c2, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

}


void
Tool::showComparison(CloudCT::Ptr c1, CloudCT::Ptr c2)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(c1, "1");
	viewer.addPointCloud(c2, "2");
	viewer.spin();
}

void
Tool::showComparison(CloudNT::Ptr c1, CloudT::Ptr c2)
{
	pcl::visualization::PCLVisualizer viewer;
	CloudT::Ptr clo1(new CloudT);
	pcl::copyPointCloud(*c1, *clo1);
	viewer.addPointCloud(clo1, "1");
	viewer.addPointCloud(c2, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "2");
	viewer.spin();
}

void
Tool::saveMatrix4f(Eigen::Matrix4f data, string filename)
{
	ofstream out;
	out.open(filename, ios::binary);
	for (int i = 0; i < 16; i++)
	{
		out.write((char*)&(data(i % 4, i / 4)), sizeof(float));
	}
	out.close();
}


Eigen::Matrix4f
Tool::readMatrix4f(string filename)
{
	ifstream in;
	in.open(filename, ios::binary);
	float cur;
	Eigen::Matrix4f ret;
	for (int i = 0; i < 16; i++)
	{
		in.read((char*)&cur, sizeof(float));
		ret(i % 4, i / 4) = cur;
	}
	return ret;
}

CloudT::Ptr
Tool::removeInvalid(CloudT::Ptr cloud)
{
	CloudT::Ptr ret(new CloudT());
	int count = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x != 0 || cloud->points[i].y != 0 || cloud->points[i].z != 0)
		{
			count++;
			ret->points.push_back(cloud->points[i]);
		}
	}
	ret->resize(count);
	ret->width = count;
	ret->height = 1;
	return ret;
}

namespace arsenal
{
	void fourierTransform(CP cloud)
	{
		CP ret(new CloudT);
		*ret = *cloud;
		for (int i = 1; i < 2; i++)
		{
			int N = cloud->size();
			Eigen::VectorXcf x = cloud->getMatrixXfMap(3, 4, 0).row(i);  // 输入信号


			// 执行傅里叶变换
			Eigen::FFT<float> fft;
			Eigen::VectorXcf spectrum = fft.fwd(x);

			//// 输出频谱
			//std::cout << "原始频谱：" << std::endl;
			//for (int i = 0; i < N; ++i) {
			//	std::cout << std::abs(spectrum(i)) << " ";
			//}
			//std::cout << std::endl;

			// 将前一半频谱置零（去除低频信号）
			spectrum.tail(10).setZero();

			//// 输出处理后的频谱
			//std::cout << "去除低频信号后的频谱：" << std::endl;
			//for (int i = 0; i < N; ++i) {
			//	std::cout << std::abs(spectrum(i)) << " ";
			//}
			//std::cout << std::endl;

			// 执行傅里叶逆变换
			Eigen::VectorXcf inverted = fft.inv(spectrum);

			//// 输出逆变换结果
			//std::cout << "逆变换结果：" << std::endl;
			//std::cout << inverted << std::endl;
			ret->getMatrixXfMap(3, 4, 0).row(i) = inverted.real();
		}
		Tool::showComparison(cloud, ret);
	}
	vector<string> pathGather(string root, string file)
	{
		vector<string> path_list;
		{
			string inPath = root + file;
			__int64 handle;
			struct _finddata_t fileinfo;
			handle = _findfirst(inPath.c_str(), &fileinfo);
			if (handle == -1)
				return path_list;

			do
			{
				string szPath;
				szPath = root + string(fileinfo.name);
				path_list.push_back(szPath);
			} while (!_findnext(handle, &fileinfo));
		}
		return path_list;
	}
	CP indices2cloud(pcl::PointIndices::Ptr inliers, CP cloud)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		extract.setInputCloud(cloud);
		extract.setIndices(inliers);

		extract.setNegative(false);

		pcl::PointCloud<pcl::PointXYZ>::Ptr extractCloud(new pcl::PointCloud<pcl::PointXYZ>);
		extract.filter(*extractCloud);
		return extractCloud;
	}
	std::vector<double> readDoubleFromFile(const std::string& filename) {
		std::vector<double> result;

		// 打开文件
		std::ifstream inputFile(filename);

		// 检查文件是否成功打开
		if (!inputFile.is_open()) {
			throw std::runtime_error(filename + "no exist");
		}

		// 逐行读取文件内容
		std::string line;
		while (std::getline(inputFile, line)) {
			std::istringstream iss(line);
			double temp;

			// 逐个读取double并添加到向量中
			while (iss >> temp) {
				result.push_back(temp);
			}
		}

		// 关闭文件
		inputFile.close();

		return result;
	}
	Eigen::Matrix3f constructRotationFromZ(Eigen::Vector3f z)
	{
		z.normalize();

		Eigen::Vector3f x;
		if (std::abs(z.x()) < std::abs(z.y())) {
			x << 1, 0, 0;
		}
		else {
			x << 0, 1, 0;
		}

		// 计算第一个垂直向量
		x = x - z.dot(x) * z;
		x.normalize();

		// 计算第二个垂直向量
		Eigen::Vector3f y = z.cross(x);
		y.normalize();

		Eigen::Matrix3f matrix;
		matrix << x, y, z;
		return matrix.transpose();
	}

	bool copyFile_czx(const std::string& sourcePath, const std::string& destinationPath) {
		std::ifstream sourceFile(sourcePath, std::ios::binary);
		std::ofstream destinationFile(destinationPath, std::ios::binary);

		if (sourceFile && destinationFile) {
			// 使用迭代器流将文件内容拷贝
			destinationFile << sourceFile.rdbuf();
			return true;
		}
		else {
			return false;
		}
	}

	LPCWSTR ConvertToLPCWSTR(const std::string& inputString)
	{
		int size_needed = MultiByteToWideChar(CP_UTF8, 0, &inputString[0], static_cast<int>(inputString.size()), NULL, 0);
		std::wstring wideString(size_needed, 0);
		MultiByteToWideChar(CP_UTF8, 0, &inputString[0], static_cast<int>(inputString.size()), &wideString[0], size_needed);
		return wideString.c_str();
	}

	void comparisonTwoCloud(CP c1, CP c2, float threshold)
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

			#pragma omp section
			{
				pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
				kdtree.setInputCloud(c2);

				for (int i = 0; i < c1->size(); i++)
				{
					// 执行KNN搜索
					int k = 1; // 选择k值
					std::vector<int> point_indices(k);
					std::vector<float> point_distances(k);
					kdtree.nearestKSearch(c1->points[i], k, point_indices, point_distances);
					if (point_distances[0] > threshold)
					{
						dif1to2->push_back(c1->points[i]);
					}
				}

			}
		}

		//pcl::io::savePCDFileBinary("c1.pcd", *c1);
		//pcl::io::savePCDFileBinary("c2.pcd", *c2);

		if (dif2to1->size() > 0)
		{
			Tool::showComparison(c1, c2, dif2to1, 2, 2, 4);
			//pcl::io::savePCDFileBinary("dif21.pcd", *dif2to1);
		}
		if (dif1to2->size() > 0)
		{
			Tool::showComparison(c1, c2, dif1to2, 2, 2, 4);
			//pcl::io::savePCDFileBinary("dif12.pcd", *dif1to2);

		}

	}

}