#ifndef CLUSTER_H
#define CLUSTER_H

#include "data/point.h"
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

    Point getMean();
    double getWidth();
    double getHeight();

    void reindex();

private:
    void calculateMean();

private:
    Point mean;
    double height;
    double width;
    bool dirty_;
};

#endif // CLUSTER_H
