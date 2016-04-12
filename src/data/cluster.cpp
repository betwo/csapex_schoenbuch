/// HEADER
#include "data/cluster.h"
#include "data/point.h"

#include <limits>

Cluster::Cluster()
    : id(0), col_start(0), col_end(0), dirty_(false)
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

    dirty_ = true;
}

void Cluster::merge(Cluster* other)
{
    for(std::vector<Point*>::iterator it = other->pts.begin(); it != other->pts.end(); ++it) {
        pts.push_back(*it);
        (*it)->cluster = this;
    }
    other->pts.clear();
    other->mean = Point();

    dirty_ = true;
}

void Cluster::reindex()
{
    for(std::vector<Point*>::iterator it = pts.begin(); it != pts.end(); ++it) {
        (*it)->cluster = this;
    }
}

void Cluster::clear()
{
    for(std::vector<Point*>::iterator it = pts.begin(); it != pts.end(); ++it) {
        (*it)->cluster = 0;
    }
    pts.clear();
    mean = Point();
}

Point Cluster::getMean()
{
    if(dirty_) {
        calculateMean();
    }
    return mean;
}

double Cluster::getWidth()
{
    if(dirty_) {
        calculateMean();
    }
    return width;
}

double Cluster::getHeight()
{
    if(dirty_) {
        calculateMean();
    }
    return height;
}

void Cluster::calculateMean()
{
    mean.x = 0.0;
    mean.y = 0.0;
    mean.z = 0.0;

    double max_x = -std::numeric_limits<double>::infinity();
    double min_x = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();

    for(std::vector<Point*>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
        const Point& pt = (**it);
        mean.x += pt.x;
        mean.y += pt.y;
        mean.z += pt.z;

        if(pt.x < min_x) {
            min_x = pt.x;
        }
        if(pt.x > max_x) {
            max_x = pt.x;
        }
        if(pt.y < min_y) {
            min_y = pt.y;
        }
        if(pt.y > max_y) {
            max_y = pt.y;
        }
        if(pt.z < min_z) {
            min_z = pt.z;
        }
        if(pt.z > max_z) {
            max_z = pt.z;
        }
    }
    mean.x /= pts.size();
    mean.y /= pts.size();
    mean.z /= pts.size();

    height = max_z - min_z;
    width = std::max(max_x - min_x, max_y - min_y);

    dirty_ = false;
}
