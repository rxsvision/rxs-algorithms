#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include<vector>
#include"czxTool.h"

using namespace std;
enum state { Fail, Success, Done };

class Ransac
{
public:
	Ransac();

	CP randSample(CP cloud);


	void fitOnce(CP sample);

	state evaluate(CP cloud);

	float p2ellipse(PointT p);

	float p2focus(PointT p);

	bool fit(CP cloud);

	//Eigen::Vector3f minProjectPlane(CP cloud);

	//CP projectPlane(CP cloud);


	CP drawEllipse();

	float getx() { return ellipse_x; };
	float gety() { return ellipse_y; };
	float geta() { return ellipse_a; };
	float getb() { return ellipse_b; };
	float gettheta() { return ellipse_theta; };

	CP fitedPoints() {
		return ellipse_found;
	}

private:
	Eigen::VectorXf coff;

	float ellipse_x;
	float ellipse_y;
	float ellipse_a;
	float ellipse_b;
	float ellipse_c;
	float ellipse_theta;
	float threshold_inlier;
	int sample_num;
	int max_iter;

	Eigen::Vector2f f1;
	Eigen::Vector2f f2;

	vector<int> valid_num;
	//vector<int> sample_index;

	int valid_sum;

	CP ellipse_found;

	CP approximate_sample;

	float approximate_inlier;
	int approximate_num;
	unordered_map<string, string> config;


};