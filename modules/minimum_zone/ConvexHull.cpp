#include "ConvexHull.h"

ConvexHull::ConvexHull(CP cloud_in)
{
    cloud = cloud_in;
    for (auto& p : *cloud)
    {
        all_vertex.push_back(p);
    }
}

bool ConvexHull::quickHull(Hull& ret)
{
    if(cloud->size()<10)
        return false;

    auto facets = initTetrahedron(cloud);

    list<PointT>::iterator tmp_iter;

    //遍历四面体每个点,遍历未分配的点,评估是否是面的外部点集
    for (auto& facet: facets)
    {
        Plane plane(*facet);
        for (auto cur_iter=all_vertex.begin(); cur_iter!=all_vertex.end();)
        {
            if (plane.isOnPositiveSide(*cur_iter))
            {
                tmp_iter = cur_iter;
                tmp_iter++;
                facet->outsideSet->splice(facet->outsideSet->end(), all_vertex, cur_iter);
                cur_iter = tmp_iter;
            }
            else
                cur_iter++;
        }
    }
    FacetList facetPendList;//待定面集
    FacetList::iterator facet_iter;
    //Facet head;
    for (facet_iter = facets.begin(); facet_iter != facets.end(); facet_iter++)
    {
        if ((*facet_iter)->outsideSet)
            facetPendList.splice(facetPendList.end(), facets, facet_iter);
    }

    // TODO: quickHull iteration not yet implemented — pending facets collected but not processed
    return false;
}

FacetList ConvexHull::initTetrahedron(CP cloud)
{
    FacetList ret;

    //int index[4];
    //cloud->getMatrixXfMap(3, 4, 0).row(0).maxCoeff(&index[0]);
    //cloud->getMatrixXfMap(3, 4, 0).row(0).minCoeff(&index[1]);
    //cloud->getMatrixXfMap(3, 4, 0).row(1).maxCoeff(&index[2]);
    //cloud->getMatrixXfMap(3, 4, 0).row(1).minCoeff(&index[3]);

    list<PointT>::iterator tetrahedron_vertex[4];
    tetrahedron_vertex[0] = tetrahedron_vertex[1] = tetrahedron_vertex[2] = all_vertex.begin();
    for (auto cur = all_vertex.begin(); cur != all_vertex.end(); cur++)
    {
        if (cur->x < tetrahedron_vertex[0]->x)
            tetrahedron_vertex[0] = cur;
        if (cur->x > tetrahedron_vertex[1]->x)
            tetrahedron_vertex[1] = cur;
        if (cur->y < tetrahedron_vertex[2]->y)
            tetrahedron_vertex[2] = cur;
        if (cur->y > tetrahedron_vertex[3]->y)
            tetrahedron_vertex[3] = cur;
    }


    PointT p[4];
    p[0] = *tetrahedron_vertex[0];
    p[1] = *tetrahedron_vertex[1];
    p[2] = *tetrahedron_vertex[2];
    p[3] = *tetrahedron_vertex[3];

    for(auto &v: tetrahedron_vertex)
        all_vertex.erase(v);

    //Facet f1(p[0], p[1], p[2]);
    //Facet f2(p[0], p[1], p[3]);
    //Facet f3(p[0], p[2], p[3]);
    //Facet f4(p[1], p[2], p[3]);

    FacetPtr f1 = std::make_shared<Facet>(p[0], p[1], p[2], p[3]);
    FacetPtr f2 = std::make_shared<Facet>(p[0], p[1], p[3], p[2]);
    FacetPtr f3 = std::make_shared<Facet>(p[0], p[2], p[3], p[1]);
    FacetPtr f4 = std::make_shared<Facet>(p[1], p[2], p[3], p[0]);

    f1->setNeighbor(*f2, *f4, *f3);
    f2->setNeighbor(*f1, *f4, *f3);
    f3->setNeighbor(*f1, *f4, *f2);
    f4->setNeighbor(*f1, *f3, *f2);

    ret.push_back(f1);
    ret.push_back(f2);
    ret.push_back(f3);
    ret.push_back(f4);

    return ret;
}



Plane::Plane(Facet face)
{
    normal = (face.vertex[1].getVector3fMap() - face.vertex[0].getVector3fMap()).cross(face.vertex[2].getVector3fMap() - face.vertex[0].getVector3fMap());
    d = -normal.dot(face.vertex[0].getVector3fMap());
}

bool Plane::isOnPositiveSide(PointT& p) const
{
    return normal.dot(p.getVector3fMap()) + d > 0;
}
