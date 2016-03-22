/// HEADER
#include "data/cluster.h"
#include "data/point.h"

Cluster::Cluster()
    : id(0), col_start(0), col_end(0)
{

}

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
    for(std::vector<Point*>::iterator it = other->pts.begin(); it != other->pts.end(); ++it) {
        pts.push_back(*it);
        (*it)->cluster = this;
    }
    other->pts.clear();
}

void Cluster::clear()
{
    for(std::vector<Point*>::iterator it = pts.begin(); it != pts.end(); ++it) {
        (*it)->cluster = 0;
    }
    pts.clear();
}
