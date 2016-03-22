#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>

struct Point;

struct Cluster
{
    Cluster();

    int id;
    int col_start;
    int col_end;

    std::vector<Point*> pts;

    void merge(Cluster* other);
    void add(Point* p);

    bool empty();

    void clear();
};

#endif // CLUSTER_H
