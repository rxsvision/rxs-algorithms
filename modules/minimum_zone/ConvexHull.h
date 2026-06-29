#pragma once
#include"czxTool.h"
#include"HullBase.h"
class Hull;
class Facet;



class Plane
{
public:
	Plane(Facet face);
	bool isOnPositiveSide(PointT& p) const;
	Vector3f normal;
	double d;
};




class ConvexHull
{
	ConvexHull(CP cloud_in);
	bool quickHull(Hull & ret);
	FacetList initTetrahedron(CP cloud);

	CP cloud;
	list<PointT> all_vertex;
};