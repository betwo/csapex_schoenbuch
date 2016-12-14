#ifndef POINT_H
#define POINT_H

class Cluster;

struct ClusteredPoint
{
    ClusteredPoint();

    float x;
    float y;
    float z;
    float intensity;

    float jump_distance;

    int row;
    int col;

    int type ;

    Cluster* cluster;

    double distanceXYZ(const ClusteredPoint& other);
    double distanceXY(const ClusteredPoint& other);
    double range();
};

#endif // POINT_H
