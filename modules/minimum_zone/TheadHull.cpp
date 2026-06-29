#include "TheadHull.h"

ThreadHull::ThreadHull(int tn, CP cloud)
{
    job_num = 0;
	thread_num = tn;
    if (tn < 4)
        throw logic_error("thread num must greater than 4");
	for (auto& p : *cloud)
	{
		points.push_back(p);
	}
    hull_points.reset(new CloudT);
}

void ThreadHull::initTetrahedron()
{
    //找出四个初始点
    list<PointT>::iterator tetrahedron_vertex[4];
    tetrahedron_vertex[0] = tetrahedron_vertex[1] = tetrahedron_vertex[2] = tetrahedron_vertex[3] = points.begin();
    for (auto cur = points.begin(); cur != points.end(); cur++)
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
    for (int i = 0; i < 4; i++)
    {
        p[i] = *tetrahedron_vertex[i];
        hull_points->push_back(p[i]);
    }

    for (auto& v : tetrahedron_vertex)
        points.erase(v);

    FacetPtr f1(new Facet(p[0], p[1], p[2], p[3]));
    FacetPtr f2(new Facet(p[0], p[1], p[3], p[2]));
    FacetPtr f3(new Facet(p[0], p[2], p[3], p[1]));
    FacetPtr f4(new Facet(p[1], p[2], p[3], p[0]));

    //f1.setNeighbor(f2, f4, f3);
    //f2.setNeighbor(f1, f4, f3);
    //f3.setNeighbor(f1, f4, f2);
    //f4.setNeighbor(f1, f3, f2);

    Pend_facets.push_back(f1);
    Pend_facets.push_back(f2);
    Pend_facets.push_back(f3);
    Pend_facets.push_back(f4);

    //遍历四面体每个点,遍历未分配的点,评估是否是面的外部点集
    PointIterator tmp_iter;
    for (auto& facet : Pend_facets)
    {
        for (auto cur_iter = points.begin(); cur_iter != points.end();)
        {
            if (facet->isOnPositiveSide(*cur_iter))
            {
                tmp_iter = cur_iter;
                tmp_iter++;
                facet->outsideSet->splice(facet->outsideSet->end(), points, cur_iter);
                cur_iter = tmp_iter;
            }
            else
                cur_iter++;
        }
    }
}

void ThreadHull::distribution(FacetPtr facet)
{
    if (facet->outsideSet->empty())
    {
        subJob();
        startNew();
        return ;
    }
    
    //找出最远点
    auto furthest = facet->furthestVertex();
    PointT furthest_point = *furthest;
    mutex_points.lock();
    hull_points->push_back(furthest_point);
    mutex_points.unlock();
    facet->outsideSet->erase(furthest);
    

    //构造新面
    FacetList new_facet;
    FacetPtr f0(new Facet(facet->vertex[0], facet->vertex[1], furthest_point, facet->vertex[2]));
    new_facet.push_back(f0);
    FacetPtr f1(new Facet(facet->vertex[0], facet->vertex[2], furthest_point, facet->vertex[1]));
    new_facet.push_back(f1);
    FacetPtr f2(new Facet(facet->vertex[1], facet->vertex[2], furthest_point, facet->vertex[0]));
    new_facet.push_back(f2);


    //分配点
    PointIterator tmp_iter;
    for (auto& f : new_facet)
    {
        for (auto cur_iter = facet->outsideSet->begin(); cur_iter != facet->outsideSet->end();)
        {
            if (f->isOnPositiveSide(*cur_iter))
            {
                tmp_iter = cur_iter;
                tmp_iter++;
                f->outsideSet->splice(f->outsideSet->end(), *facet->outsideSet, cur_iter);
                cur_iter = tmp_iter;
            }
            else
                cur_iter++;
        }
    }

    mutex_pend_facets.lock();
    Pend_facets.insert(Pend_facets.end(), new_facet.begin(), new_facet.end());
    mutex_pend_facets.unlock();
    subJob();
    startNew();
}

void ThreadHull::process()
{
    initTetrahedron();
    mutex_pend_facets.lock();
    auto it = Pend_facets.begin();
    while (it!=Pend_facets.end())
    {
        addJob();
        std::thread backgroundThread(&ThreadHull::distribution, this, *it);
        backgroundThread.detach();
        it = Pend_facets.erase(it);
    }
    mutex_pend_facets.unlock();

    unique_lock<mutex> lock(mutex_main);
    cv.wait(lock);
}

void ThreadHull::startNew()
{
    mutex_pend_facets.lock();
    while (!Pend_facets.empty()&&job_num<thread_num)
    {
        addJob();
        std::thread backgroundThread(&ThreadHull::distribution, this, *Pend_facets.begin());
        backgroundThread.detach();
        Pend_facets.pop_front();
    }
    mutex_pend_facets.unlock();

    if (job_num == 0 && Pend_facets.empty())
    {
        {
            std::lock_guard<std::mutex> lock(mutex_main);
        }
        cv.notify_one();
    }

}
