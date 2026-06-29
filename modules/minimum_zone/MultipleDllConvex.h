#pragma once
#include"czxTool.h"

class MultipleDllConvex
{
public:
	MultipleDllConvex(int thread_num_in);
	~MultipleDllConvex();
	void setInputCloud(CP cloud_in);

	CP solveConvexHull(CP clo, int id);

	vector<CP> devideCloud();

	CP process();

public:
	CP cloud;
	int thread_num;

	typedef CP(__cdecl* CC)(CP cloud_);
	vector<CC> func_list;
	vector<HMODULE> loaded_dll;
};

