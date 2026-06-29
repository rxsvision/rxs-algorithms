#pragma once

#include"czxTool.h"
#include<pcl/kdtree/kdtree_flann.h>
#include"CzxRansac.h"

class Hole
{
public:
	unordered_map<string, string> config;

	Hole()
	{
		config = czxTool::readProfile("conf.czx");
	};

	CP KNNBoundExtract(CP cloud);

	PointT centerFit(CP cloud);


	//template <typename CLOUD> void fun(CLOUD cloud);

	//template <typename CLOUD> PointT process(CLOUD cloud);
};

