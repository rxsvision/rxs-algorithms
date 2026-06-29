#pragma once

#include"czxTool.h"


class Tree
{
public:
	Tree();
	Tree(int div_dim);
	~Tree();

public:
	CP val;
	int div_dim;
	Tree* left;
	Tree* right;
	float min;
	float max;

	int min_index;
	int max_index;
};

class SquareTree
{
public:
	SquareTree();
	Tree tree;
	CP data;
	int max_leaf;
	void build(CP& cloud);
	void build_x(CP& cloud, Tree& root, int div_count = 1);
	void build_y(CP& cloud, Tree& root, int div_count = 1);

	void build_x_v2(Tree& root);//重新排序点云
	void build_y_v2(Tree& root);//重新排序点云

	template<typename CloudType>
	int squareSearch(PointT& target, float* radius, CloudType& ret);
	void squareSearch(float* min, float* max, Tree& root, CP& ret);
	template<typename CloudType>
	void squareSearch_v2(float* min, float* max, Tree& root, CloudType& ret);
	template<typename CloudType>
	void add(CloudType& cloud, PointT& p);
};



template<typename CloudType>
int SquareTree::squareSearch(PointT& target_, float* radius, CloudType& ret)
{
	//CP ret(new CloudT);
	float target[2];
	target[0] = target_.x;
	target[1] = target_.y;
	float min[2];
	float max[2];
	min[0] = target[0] - radius[0];
	min[1] = target[1] - radius[1];
	max[0] = target[0] + radius[0];
	max[1] = target[1] + radius[1];

	//squareSearch(min, max, tree, ret);
	//squareSearch_v2(min, max, tree, ret);
	squareSearch_v2(min, max, tree, ret);
	return ret->size();
}

template<typename CloudType>
void SquareTree::squareSearch_v2(float* min_, float* max_, Tree& root, CloudType& ret)
{
	float& min = min_[root.div_dim];
	float& max = max_[root.div_dim];
	if (root.left && root.left->max > min && root.left->min < max)
	{
		squareSearch_v2(min_, max_, *root.left, ret);
	}
	if (root.right && root.right->max > min && root.right->min < max)
	{
		squareSearch_v2(min_, max_, *root.right, ret);
	}
	if (!root.left && !root.right)
	{
		for (auto cur = data->begin() + root.min_index; cur != data->begin() + root.max_index; cur++)
		{
			PointT& p = *cur;
			if (p.x > min_[0] && p.x<max_[0] && p.y> min_[1] && p.y < max_[1])
				add(ret, p);
		}
	}
}

template<typename CloudType>
void SquareTree::add(CloudType& cloud, PointT& p)
{
	cloud->push_back(p);
}