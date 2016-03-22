/// HEADER
#include "data/point.h"

/// SYSTEM
#include <cmath>

Point::Point()
    : jump_distance(0.0f), type(0), cluster(0)
{

}

double Point::distanceXYZ(const Point& other)
{
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double Point::distanceXY(const Point& other)
{
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx*dx + dy*dy);
}

double Point::range()
{
    return std::sqrt(x*x + y*y + z*z);
}
