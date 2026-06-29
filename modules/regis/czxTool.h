//#pragma once
#ifndef CZXTOOL_HPP
#define CZXTOOL_HPP
#include<map>
#include<string>
#include<fstream>
#include <mutex>
#include<Eigen/Dense>
#include<iostream>
#include <time.h>
#include <unordered_map>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/extract_indices.h>


//#include<ccLog.h>
//#include <QTextStream>
//#include <QBuffer>
//#include <QDebug>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointCT> CloudCT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> CloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef CloudT::Ptr CP;

using namespace std;
using namespace Eigen;

class CzxLog {
public:
	CzxLog(string path = "")
	{
		if (!path.empty())
		{
			fileStream.open(path, std::ios::app);

			// 检查文件是否成功打开
			if (fileStream.is_open()) {
			}
		}
	}

	~CzxLog() {
		fileStream.close();
	}

	template <typename T>
	CzxLog& operator<<(const T& value) {
		//std::stringstream stream;
		//stream << value;
		//printf("%s", stream.str());
		//ccLog::Print(QString::fromStdString(stream.str()));

		//控制台应用
		cout << value;

		if(fileStream.is_open())
			fileStream << value;
		return *this;
	}

	// 添加换行符并刷新缓冲区
	CzxLog& operator<<(std::ostream& (*manip)(std::ostream&)) {
		//if (manip != static_cast<std::ostream & (*)(std::ostream&)>(std::endl))
		//{
		//	ccLog::Print(QStringLiteral("未实现流的指针"));
		//}
		//else {
		//	ccLog::Print("\n");
		//}


		//控制台窗口
		manip(cout);

		if (fileStream.is_open()) {
			manip(fileStream);
		}
		return *this;
	}

private:
	ofstream fileStream;
};

class CzxTimer {
public:
	CzxTimer(string me) :log(path)
	{
		// 在构造函数中获取初始时间点
		start = clock();
		this->method = me;
	}

	~CzxTimer() {
		// 在析构函数中获取当前时间点
		auto ends = clock();

		// 打印时间差
		log << "执行" << method << "经过的时间: " << ends - start << " 毫秒" << std::endl;
		
	}

	static string path;

private:
	clock_t start;
	string method;
	CzxLog log;
};

class czxTool
{
public:
	static map<string, string> readDict(string path)
	{
		ifstream in;
		in.open(path);
		string line;
		map<string, string> ret;
		while (getline(in, line)) 
		{
			int index = line.find(":");
			ret[line.substr(0, index)] = line.substr(index + 1);
		}
		return ret;
	}

	static bool saveMatrix4f(Matrix4f* matrixs, int num)
	{
		ofstream o;
		o.open("matrix.txt");
		for (int i = 0; i < num; i++)
		{
			for (int row = 0; row < 4; row++)
			{
				for (int column = 0; column < 4; column++)
				{
					o << matrixs[i].row(row)[column];
					o << "     ";
				}
				o << '\n';
			}
		}
		o.close();
		return true;
	}

	static Matrix4f* loadMatrix4f(int num, string filename)
	{
		Matrix4f *ret;
		ret = new Matrix4f[num];
		ifstream in;
		in.open(filename);
		for (int i = 0; i < num; i++)
		{
			for (int row = 0; row < 4; row++)
			{
				for (int column = 0; column < 4; column++)
				{
					in >> ret[i].row(row)[column];
				}
			}
		}
		return ret;
	}
	
	static Matrix4f getTranslate(Matrix4f mat)
	{
		Matrix4f ret = Matrix4f::Identity();
		//Vector4f bias = mat.col(3);
		ret.block<4, 1>(0, 3) = mat.col(3);
		return ret;
	}

	static Matrix4f separateTranslate(Matrix4f mat)
	{
		mat.col(3) = Vector4f(0, 0, 0, 1);
		return mat;
	}

	static void printMatrixXf(MatrixXf m)
	{
		int rows = m.rows();
		int cols = m.cols();
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < cols; c++)
			{
				CzxLog() << m.row(r)[c] << "    ";
			}
			CzxLog() << std::endl;
		}
	}

	static unordered_map<string, string> readProfile(string path)
	{
		unordered_map<string, string> map;

		std::ifstream configFile(path); // 打开参数文件
		if (!configFile.is_open())
		{
			CzxLog() << "没有读取到配置文件" << path  << std::endl;
			return map;
		}
		std::string line;
		while (std::getline(configFile, line)) {
			// 使用适当的方法解析每一行，这里假设每行的格式为 "属性名=属性值"
			size_t delimiterPos = line.find('=');
			if (delimiterPos != std::string::npos) {
				std::string key = line.substr(0, delimiterPos);
				std::string value = line.substr(delimiterPos + 1);
				map[key] = value;
			}
		}

		configFile.close(); // 关闭文件
		if (map.size() == 0)
			CzxLog() << "没有读取到配置文件:" << path << "\n";
		return map;
	}

};

class Tool
{
public:
	void selectWindow(string file_name);

	static void show(CloudT::Ptr clo);
	void show(CloudNT::Ptr clo);
	static void showComparison(CloudT::Ptr c1, CloudT::Ptr c2);
	static void showComparison(CloudT::Ptr c1, CloudT::Ptr c2, int size1, int size2, function<void(const pcl::visualization::KeyboardEvent&)> callback = nullptr, string name = "None");
	static void showComparison(CloudT::Ptr c1, PointT c2, int size1=1, int size2=3, function<void(const pcl::visualization::KeyboardEvent&)> callback = nullptr, string name = "None");
	static void showComparison(CloudT::Ptr c1, CloudT::Ptr c2, CloudT::Ptr c3, int size1=1, int size2=1, int size3=1, function<void(const pcl::visualization::KeyboardEvent&)> callback = nullptr, string name = "None");

	static void showComparison(CloudT::Ptr c1, CloudT::Ptr c2, bool coor);
	void showComparison(CloudNT::Ptr c1, CloudT::Ptr c2);
	void showComparison(CloudCT::Ptr c1, CloudCT::Ptr c2);

	void saveMatrix4f(Eigen::Matrix4f data, string filename);
	Eigen::Matrix4f readMatrix4f(string filename);

	static CloudT::Ptr removeInvalid(CloudT::Ptr cloud);
	
private:
	pcl::visualization::PCLVisualizer* viewer;
	static std::mutex mtx_showComparison_ccss; // 创建互斥锁
};

struct CloudAndView
{
	pcl::visualization::PCLVisualizer* v;
	CloudT::Ptr c;
};

class CzxComparison
{
public:
	CzxComparison(CP cloud, int size1=1, int size2=1):before(new CloudT), after(new CloudT), s1(size1), s2(size2)
	{
		*before = *cloud;
		after = cloud;

	}
	~CzxComparison()
	{
		Tool::showComparison(before, after, s1, s2);
	}
private:
	CP before;
	CP after;
	int s1, s2;
};

class threadKey
{
private:
	int max = 0;
	int index = 0;
	mutex m;
public:
	threadKey(int max_) {
		max = max_;
	}
	int getKey()
	{
		int ret;
		m.lock();
		ret = index;
		index++;
		m.unlock();
		return ret;
	}
};

namespace arsenal {
	void fourierTransform(CP cloud);
	//root是目录路径,file是文件名,记得最后带\ 
	vector<string> pathGather(string root, string file = "");
	CP indices2cloud(pcl::PointIndices::Ptr inliers, CP cloud);
	std::vector<double> readDoubleFromFile(const std::string& filename);
};

#endif