#pragma once
#include"czxTool.h"
#include"HullBase.h"

class ThreadHull
{
public:
	ThreadHull(int tn, CP cloud);
	void initTetrahedron();
	void distribution(FacetPtr facet);
	void process();
	void startNew();

	void addJob() 
	{ 
		mutex_job.lock(); 
		job_num++;
		mutex_job.unlock();
	};
	void subJob()
	{
		mutex_job.lock();
		job_num--;
		mutex_job.unlock();
	};

public:
	PointList points;
	FacetList Pend_facets;
	CP hull_points;

private:
	int job_num;
	int thread_num;
	std::mutex mutex_job;
	std::mutex mutex_points;
	std::mutex mutex_pend_facets;
	std::condition_variable cv;
	std::mutex mutex_main;
};

