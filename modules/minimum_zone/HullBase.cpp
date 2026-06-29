#include "HullBase.h"
Facet::~Facet()
{
    if (outsideSet != nullptr) {
        delete outsideSet;
        outsideSet = nullptr;
    }
}

//Facet::Facet(PointT a, PointT b, PointT c)
//{
//    vertex[0] = a;
//    vertex[1] = b;
//    vertex[2] = c;
//    neighbor.resize(3);
//    outsideSet = new PointList();
//}

Facet::Facet(PointT a, PointT b, PointT c, PointT target)
{
    normal = (a.getVector3fMap() - b.getVector3fMap()).cross(a.getVector3fMap() - c.getVector3fMap());
    normal.normalize();
    if (normal.dot(target.getVector3fMap()) > 0)
    {
        swap(b, c);
        normal = -normal;
    }

    //normal = (a.getVector3fMap() - b.getVector3fMap()).cross(a.getVector3fMap() - c.getVector3fMap());
    //normal.normalize();

    d = -normal.dot(a.getVector3fMap());
    vertex[0] = a;
    vertex[1] = b;
    vertex[2] = c;
    neighbor.resize(3);
    outsideSet = new PointList();
}

Facet::Facet(PointT a, PointT b, PointT c, Vector3f reference)
{
    normal = (a.getVector3fMap() - b.getVector3fMap()).cross(a.getVector3fMap() - c.getVector3fMap());
    normal.normalize();
    if (normal.dot(reference) < 0)
    {
        swap(b, c);
        normal = -normal;
    }
    d = -normal.dot(a.getVector3fMap());
    vertex[0] = a;
    vertex[1] = b;
    vertex[2] = c;
    neighbor.resize(3);
    outsideSet = new PointList();
}

PointIterator Facet::furthestVertex()
{
    PointIterator furthest = outsideSet->begin();
    //Vector3f normal = (vertex[0].getVector3fMap() - vertex[1].getVector3fMap()).cross(vertex[0].getVector3fMap() - vertex[2].getVector3fMap());

    //这里不是真正的距离,因为没有考虑d的存在
    double dis_max = normal.dot(furthest->getVector3fMap());
    for (PointIterator iter = outsideSet->begin(); iter != outsideSet->end(); iter++)
    {
        double dis = normal.dot(iter->getVector3fMap());
        if (dis > dis_max)
        {
            dis_max = dis;
            furthest = iter;
        }
    }
    return furthest;
}

void Facet::init(PointT a, PointT b, PointT c)
{
    vertex[0] = a;
    vertex[1] = b;
    vertex[2] = c;
    neighbor.resize(3);
    outsideSet = new PointList();
}

void Facet::setNeighbor(Facet f0, Facet f1, Facet f2)
{
    neighbor[0] = f0;
    neighbor[1] = f1;
    neighbor[2] = f2;
}

bool Facet::isOnPositiveSide(PointT& p) const
{
    return normal.dot(p.getVector3fMap()) + d > 0;
}
