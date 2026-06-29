#pragma once
#include"czxTool.h"

class Facet;

typedef shared_ptr<Facet> FacetPtr;
typedef list<FacetPtr> FacetList;
typedef FacetList::iterator FacetListIterator;
typedef vector<Facet> FacetVector;
typedef list<PointT> PointList;
typedef PointList::iterator PointIterator;


class Facet
{
public:
	Facet() { outsideSet = nullptr; };
	~Facet();
	//Facet(PointT a, PointT b, PointT c);
	Facet(PointT a, PointT b, PointT c, PointT target);//要求新面在目标点的上方
	Facet(PointT a, PointT b, PointT c, Vector3f reference);//要求新面和参考法线夹角小于90
	PointIterator furthestVertex();
	void init(PointT a, PointT b, PointT c);
	void setNeighbor(Facet f0, Facet f1, Facet f2);
	bool isOnPositiveSide(PointT& p) const;


	Vector3f normal;
	double d; // 平面方程  nx+d 
	PointT vertex[3];
	FacetVector neighbor;
	PointList* outsideSet;


};

