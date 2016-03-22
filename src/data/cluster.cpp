/// HEADER
#include "data/cluster.h"
#include "data/point.h"

bool Cluster::empty()
{
    return pts.empty();
}

void Cluster::add(Point *p)
{
    pts.push_back(p);
    p->cluster = this;
}

void Cluster::merge(Cluster* other)
{
    for(Point* p : other->pts) {
        pts.push_back(p);
        p->cluster = this;
    }
    other->pts.clear();
}

void Cluster::clear()
{
    for(Point* p : pts) {
        p->cluster = nullptr;
    }
    pts.clear();
}
