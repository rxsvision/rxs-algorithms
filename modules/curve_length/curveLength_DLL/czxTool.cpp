#include"czxTool.h"
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/filters/conditional_removal.h>


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
	void visualizePointCloudWithLines(const CP cloud) {
		// 创建 PCL 可视化对象
		pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
		// 添加点云到可视化对象
		viewer.addPointCloud(cloud, "cloud");
		// 创建 PCL 的线段提取对象
		pcl::PointCloud<pcl::PointXYZ>::Ptr lines(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t i = 0; i < 0 + 1000; ++i) {
		//for (size_t i = 0; i < cloud->size()-1; ++i) {

			//lines->push_back(cloud->points[i]);
			//lines->push_back(cloud->points[i + 1]);
			std::ostringstream os;
			os << "line_" << "_" << i;
			viewer.addLine(cloud->points[i], cloud->points[i + 1],1,0,0, os.str());
		}
		// 显示可视化
		viewer.spin();
	}

	// 按中心的角度对点云排序,计算中心点极耗算力，如果可以的话，自己设一个
	void sortByAngle(CP cloud_in, vector<PointT> centerPoints, int empty_dim)
	{
		//Eigen::VectorXf rowMean = cloud_in->getMatrixXfMap(3, 4, 0).rowwise().mean();
		//centerPoints.push_back(PointT(rowMean[0], rowMean[1], rowMean[2]));

		if (centerPoints.size() == 0)
		{
			PointT minPoint;
			PointT maxPoint;
			pcl::getMinMax3D(*cloud_in, minPoint, maxPoint);

			float diff[] = { maxPoint.x - minPoint.x , maxPoint.y - minPoint.y, maxPoint.z - minPoint.z };
			
			int region_num;
			if (diff[0] > diff[2] && diff[2] > diff[1])
			{
				region_num = static_cast<int>(std::floor(diff[0] / diff[2]));
				float step = diff[0] / region_num;
				#pragma omp parallel for
				for (int i = 0; i < region_num; i++)
				{
					pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
					pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
					condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, minPoint.x+i* step)));
					condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, minPoint.x + i * step + step)));
					condrem.setCondition(condition);
					condrem.setInputCloud(cloud_in);
					CP tmp(new CloudT);
					condrem.filter(*tmp);
					Eigen::VectorXf rowMean = tmp->getMatrixXfMap(3, 4, 0).rowwise().mean();

					#pragma omp critical
					centerPoints.push_back(PointT(rowMean[0], rowMean[1], rowMean[2]));
				}

				sort(centerPoints.begin(), centerPoints.end(), [](PointT a, PointT b) {return a.x < b.x; });
			}
			else if (diff[0] > diff[1] && diff[1] > diff[2])
			{
				region_num = static_cast<int>(std::floor(diff[0] / diff[1]));
				float step = diff[0] / region_num;
				#pragma omp parallel for
				for (int i = 0; i < region_num; i++)
				{
					pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
					pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
					condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, minPoint.x + i * step)));
					condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, minPoint.x + i * step + step)));
					condrem.setCondition(condition);
					condrem.setInputCloud(cloud_in);
					CP tmp(new CloudT);
					condrem.filter(*tmp);
					Eigen::VectorXf rowMean = tmp->getMatrixXfMap(3, 4, 0).rowwise().mean();

					#pragma omp critical
					centerPoints.push_back(PointT(rowMean[0], rowMean[1], rowMean[2]));
				}
				sort(centerPoints.begin(), centerPoints.end(), [](PointT a, PointT b) {return a.x > b.x; });
			}
			else
			{
				throw logic_error("No implement");
			}
		}


		//CP tmp(new CloudT);
		//for (auto p : centerPoints)
		//{
		//	tmp->push_back(p);
		//}
		//Tool::showComparison(cloud_in, tmp, 1, 3);

		function<bool(PointT, PointT)> compare;
		if (empty_dim == 1)
		{
			compare = [&centerPoints](PointT a, PointT b) {
				PointT c;
				bool found = false;
				double mid_x = (a.x + b.x) / 2;
				for (int i = 0; i < centerPoints.size(); i++)
				{
					if (mid_x < centerPoints[i].x)
					{
						c = centerPoints[i];
						found = true;
						break;
					}
				}
				if (!found) c = centerPoints[centerPoints.size() - 1];
				return (((a.x - c.x) * (b.z - c.z) - (a.z - c.z) * (b.x - c.x)) >= 0);
			};
		}
		//else if (empty_dim == 2)
		//{
		//	compare = [&c](PointT a, PointT b) {
		//		return (((a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)) >= 0);
		//	};
		//}
		//else if (empty_dim == 0)
		//{
		//	compare = [&c](PointT a, PointT b) {
		//		return (((a.y - c.y) * (b.z - c.z) - (a.z - c.z) * (b.y - c.y)) >= 0);
		//	};
		//}
		else
		{
			throw logic_error("No implement");
		}

		//Tool::showComparison(cloud_in, c);
		sort(cloud_in->begin(), cloud_in->end(), compare);
		visualizePointCloudWithLines(cloud_in);
	}

	//计算三个点的夹角（AB, BC）
	float computeAngleBetweenPoints2D(const pcl::PointXYZ& A, const pcl::PointXYZ& B, const pcl::PointXYZ& C, int empty_dim) {
		PointT v1(B.x - A.x, B.y - A.y, B.z - A.z);
		PointT v2(C.x - B.x, C.y - B.y, C.z - B.z);
		
		double x1, y1, x2, y2;
		if (empty_dim == 1)
		{
			x1 = v1.x;
			y1 = v1.z;
			x2 = v2.x;
			y2 = v2.z;
		}
		else if ((empty_dim == 2))
		{
			x1 = v1.x;
			y1 = v1.y;
			x2 = v2.x;
			y2 = v2.y;
		}
		else if ((empty_dim == 3))
		{
			x1 = v1.y;
			y1 = v1.z;
			x2 = v2.y;
			y2 = v2.z;
		}

		double dot_product = x1 * x2 + y1 * y2;
		double magnitude_AB = std::sqrt(x1 * x1 + y1 * y1) * std::sqrt(x2 * x2 + y2 * y2);

		double cosine = dot_product / magnitude_AB;
		double angleRadians = std::acos(cosine);
		double angleDegrees = angleRadians * 180.0 / M_PI;

		if (angleDegrees > 90.0) angleDegrees = 180 - angleDegrees;
		return angleDegrees;		
	}

	//拆分多边形
	vector<CP> splitPolygon(CP cloud_in, double angle_threshold, double dis_threshold)
	{
		vector<CP> ret;
		CP cloud_cur(new CloudT);
		sortByAngle(cloud_in);

		CP corner(new CloudT);
		#pragma omp parallel for
		for (int i = 0; i < cloud_in->size() - 3; i++)
		{
			PointT last = cloud_in->points[i];
			PointT cur = cloud_in->points[i+1];
			PointT next = cloud_in->points[i+2];
			double ang = computeAngleBetweenPoints2D(last, cur, next);

			if (ang < angle_threshold || pcl::euclideanDistance(last, cur) > dis_threshold || pcl::euclideanDistance(next, cur) > dis_threshold)
			{
				continue;
			}
			else
			{
				#pragma omp critical
				{
					corner->push_back(cur);
				}
			}
		}
		Tool::showComparison(cloud_in, corner,1,3);


		PointT last = cloud_in->points[0];
		PointT cur = cloud_in->points[1];
		PointT next = cloud_in->points[2];

		cloud_cur->push_back(last);
		cloud_cur->push_back(cur);

		for (int i = 1; i < cloud_in->size() - 2; i++)
		{
			next = cloud_in->points[i+1];
			double ang = computeAngleBetweenPoints2D(last, cur, next);

			if (computeAngleBetweenPoints2D(last, cur, next) < angle_threshold)
			{
				cloud_cur->push_back(next);
				last = cur;
				cur = next;
			}
			else {
				//下一个点是噪点
				if (computeAngleBetweenPoints2D(last, cur, cloud_in->points[i + 2]) < angle_threshold)
				{
					cloud_cur->push_back(cloud_in->points[i + 2]);
					last = cur;
					cur = cloud_in->points[i + 2];
					i++;
				}
				else {
					ret.push_back(cloud_cur);
					cloud_cur.reset(new CloudT());
					cloud_cur->push_back(cur);
					cloud_cur->push_back(next);
					last = cur;
					cur = next;
				}
			}
		}

		#undef last
		#undef cur
		#undef next

		return ret;
	}
}
