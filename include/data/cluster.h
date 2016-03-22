#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>

struct Point;

struct Cluster
{
    int id = 0;
    int col_start = 0;
    int col_end = 0;

    std::vector<Point*> pts;

    void merge(Cluster* other);
    void add(Point* p);

    bool empty();

    void clear();
};

#endif // CLUSTER_H
