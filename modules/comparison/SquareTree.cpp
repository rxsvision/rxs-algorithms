#include "SquareTree.h"

#define cpe cloud->points[cloud->size()-1]
#define cp(i) cloud->points[i]

Tree::Tree()
{
	val.reset(new CloudT);
	left = nullptr;
	right = nullptr;
}

Tree::Tree(int dim)
{
	val.reset(new CloudT);
	left = nullptr;
	right = nullptr;
	div_dim = dim;
}

Tree::~Tree()
{
	if(left)
		delete left;
	if(right)
		delete right;
}

SquareTree::SquareTree()
{
	max_leaf = 20;
}

void SquareTree::build(CP& cloud)
{
	//build_x(cloud, tree);
	data.reset(new CloudT);
	*data = *cloud;
	tree.min_index = 0;
	tree.max_index = cloud->size();
	build_x_v2(tree);
}

void SquareTree::build_x(CP& cloud, Tree& root, int div_count)
{
	if (cloud->size() <= max_leaf) return;
	float mid = (cp(0).x + cpe.x) / 2;
	Tree *left_tree = new Tree();
	Tree *right_tree = new Tree();
	float min_left = mid, max_left = mid;
	float min_right = mid, max_right = mid;
	for (auto& p : *cloud)
	{
		if (p.x < mid)
		{
			left_tree->val->push_back(p);
			if (p.x < min_left)
				min_left = p.x;
			//else if (p.x > max_left)
			//	max_left = p.x;
		}
		else
		{
			right_tree->val->push_back(p);
			//if (p.x < min_right)
			//	min_right = p.x;
			if (p.x > max_right)
				max_right = p.x;
		}
	}

	left_tree->min = min_left;
	left_tree->max = max_left;
	right_tree->min = min_right;
	right_tree->max = max_right;
	root.left = left_tree;
	root.right = right_tree;
	root.div_dim = 0;

	root.val = nullptr;
	if (div_count <= 1)
	{
		//build_y(left_tree->val, *left_tree);
		//build_y(right_tree->val, *right_tree);
		#pragma omp parallel
		{
			#pragma omp sections
			{
				#pragma omp section
				{
					build_y(left_tree->val, *left_tree);
				}

				#pragma omp section
				{
					build_y(right_tree->val, *right_tree);
				}
			}
		}
	}
	else 
	{
		build_x(left_tree->val, *left_tree, div_count - 1);
		build_x(right_tree->val, *right_tree, div_count - 1);
	}
}

void SquareTree::build_y(CP& cloud, Tree& root, int div_count)
{
	if (cloud->size() <= max_leaf) return;
	float mid = (cp(0).y + cpe.y) / 2;
	Tree* left_tree = new Tree();
	Tree* right_tree = new Tree();
	float min_left = mid, max_left = mid;
	float min_right = mid, max_right = mid;
	for (auto& p : *cloud)
	{
		if (p.y < mid)
		{
			left_tree->val->push_back(p);
			if (p.y < min_left)
				min_left = p.y;
			//if (p.y > max_left)
			//	max_left = p.y;
		}
		else
		{
			right_tree->val->push_back(p);
			//if (p.y < min_right)
			//	min_right = p.y;
			if (p.y > max_right)
				max_right = p.y;
		}
	}
	left_tree->min = min_left;
	left_tree->max = max_left;
	right_tree->min = min_right;
	right_tree->max = max_right;
	root.left = left_tree;
	root.right = right_tree;
	root.div_dim = 1;

	root.val = nullptr;
	if (div_count <= 1)
	{
		//build_x(left_tree->val, *left_tree);
		//build_x(right_tree->val, *right_tree);
		#pragma omp parallel
		{
			#pragma omp sections
			{
				#pragma omp section
				{
					build_x(left_tree->val, *left_tree);
				}

				#pragma omp section
				{
					build_x(right_tree->val, *right_tree);
				}
			}
		}
	}
	else 
	{
		build_y(left_tree->val, *left_tree, div_count - 1);
		build_y(right_tree->val, *right_tree, div_count - 1);
	}
}

void SquareTree::build_x_v2(Tree& root)
{
	if (root.max_index - root.min_index < max_leaf)
		return;

	float mid = (data->points[root.min_index].x + data->points[root.max_index - 1].x) / 2;

	auto left_iter = data->begin() + root.min_index;
	auto right_iter = data->begin() + root.max_index - 1;
	float min_val = mid;
	float max_val = mid;
	for (;;)
	{
		while (left_iter->x < mid && left_iter != data->begin() + root.max_index)
		{
			if (left_iter->x < min_val)
				min_val = left_iter->x;
			left_iter++;
		}
		if (left_iter->x > max_val)
			max_val = left_iter->x;
		while (right_iter->x >= mid && right_iter != data->begin() + root.min_index)
		{
			if (right_iter->x > max_val)
				max_val = right_iter->x;
			right_iter--;
		}
		if (right_iter->x < min_val)
			min_val = right_iter->x;
		if (left_iter >= right_iter)
			break;
		std::swap(*left_iter, *right_iter);
		left_iter++;
		right_iter--;
	}

	Tree* left = new Tree();
	Tree* right = new Tree();
	left->min_index = root.min_index;
	left->max_index = left_iter - data->begin();
	left->min = min_val;
	left->max = mid;
	right->min_index = left_iter - data->begin();
	right->max_index = root.max_index;
	right->min = mid;
	right->max = max_val;
	root.div_dim = 0;
	root.left = left;
	root.right = right;

	#pragma omp parallel
	{
		#pragma omp sections
		{
			#pragma omp section
			build_y_v2(*left);
			#pragma omp section
			build_y_v2(*right);
		}
	}
}

void SquareTree::build_y_v2(Tree& root)
{
	if (root.max_index - root.min_index < max_leaf)
		return;

	float mid = (data->points[root.min_index].y + data->points[root.max_index - 1].y) / 2;

	auto left_iter = data->begin() + root.min_index;
	auto right_iter = data->begin() + root.max_index - 1;
	float min_val = mid;
	float max_val = mid;
	for (;;)
	{
		while (left_iter->y < mid && left_iter != data->end())
		{
			if (left_iter->y < min_val)
				min_val = left_iter->y;
			left_iter++;
		}
		if(left_iter->y > max_val)
			max_val = left_iter->y;
		while (right_iter->y >= mid && right_iter != data->begin())
		{
			if (right_iter->y > max_val)
				max_val = right_iter->y;
			right_iter--;
		}
		if (right_iter->y < min_val)
			min_val = right_iter->y;
		if (left_iter >= right_iter)
			break;
		std::swap(*left_iter, *right_iter);
		left_iter++;
		right_iter--;
	}

	Tree* left = new Tree();
	Tree* right = new Tree();
	left->min_index = root.min_index;
	left->max_index = left_iter - data->begin();
	left->min = min_val;
	left->max = mid;
	right->min_index = left_iter - data->begin();
	right->max_index = root.max_index;
	right->min = mid;
	right->max = max_val;
	root.div_dim = 1;
	root.left = left;
	root.right = right;
	#pragma omp parallel
	{
		#pragma omp sections
		{
			#pragma omp section
			build_x_v2(*left);
			#pragma omp section
			build_x_v2(*right);
		}
	}
}

void SquareTree::squareSearch(float* min_, float* max_, Tree& root, CP& ret)
{
	float& min = min_[root.div_dim];
	float& max = max_[root.div_dim];
	if (root.left&&root.left->max > min && root.left->min < max)
	{
		squareSearch(min_, max_, *root.left, ret);
	}
	if (root.right && root.right->max > min && root.right->min < max)
	{
		squareSearch(min_, max_, *root.right, ret);
	}
	if (root.val)
	{
		for (auto p : *root.val)
		{
			if (p.x> min_[0] && p.x<max_[0] && p.y> min_[1] && p.y<max_[1])
				add(ret, p);
		}
	}
}
