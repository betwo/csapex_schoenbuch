#ifndef POINT_H
#define POINT_H

class Cluster;

struct Point
{
    Point();

    float x;
    float y;
    float z;
    float intensity;

    float jump_distance;

    int row;
    int col;

    int type ;

    Cluster* cluster;

    double distanceXYZ(const Point& other);
    double distanceXY(const Point& other);
    double range();
};

#endif // POINT_H
