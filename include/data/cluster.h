#ifndef CLUSTER_H
#define CLUSTER_H

#include "data/point.h"
#include <vector>

struct ClusteredPoint;

struct Cluster
{
    Cluster();

    int id;
    int col_start;
    int col_end;

    std::vector<ClusteredPoint*> pts;

    void merge(Cluster* other);
    void add(ClusteredPoint* p);

    bool empty();

    void clear();

    ClusteredPoint getMean();
    double getWidth();
    double getHeight();

    void reindex();

private:
    void calculateMean();

private:
    ClusteredPoint mean;
    double height;
    double width;
    bool dirty_;
};

#endif // CLUSTER_H
