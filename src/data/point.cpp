/// HEADER
#include <data/point.h>

/// SYSTEM
#include <cmath>

ClusteredPoint::ClusteredPoint()
    : jump_distance(0.0f), type(0), cluster(0)
{

}

double ClusteredPoint::distanceXYZ(const ClusteredPoint& other)
{
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double ClusteredPoint::distanceXY(const ClusteredPoint& other)
{
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx*dx + dy*dy);
}

double ClusteredPoint::range()
{
    return std::sqrt(x*x + y*y + z*z);
}
