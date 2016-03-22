#ifndef POINT_H
#define POINT_H

class Cluster;

struct Point
{
    float x;
    float y;
    float z;
    float intensity;

    float jump_distance = 0.0f;

    int row;
    int col;

    int type = 0;

    Cluster* cluster = nullptr;

    double distanceXYZ(const Point& other);
    double distanceXY(const Point& other);
    double range();
};

#endif // POINT_H
