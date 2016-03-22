#ifndef PILLAREXTRACTOR_H
#define PILLAREXTRACTOR_H

/// PROJECT
#include "pillar.h"
#include "data/point.h"
#include "data/cluster.h"

/// SYSTEM
#include <tf/tf.h>
#include <memory>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PillarExtractor
{
public:
    PillarExtractor();
    ~PillarExtractor();

    std::vector<Pillar> findPillars(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input);

public:
    double cluster_distance_ring_;
    double cluster_distance_vertical_;

    int cluster_min_size_;
    int cluster_max_size_;

    double cluster_max_diameter_;

    double pillar_min_intensity_;
    int pillar_min_points_;

    double pillar_radius_;
    double pillar_radius_fuzzy_;

private:

    double fitCylinder(const std::vector<Point*>& cluster,
                       double& r, Eigen::Vector3d& C, Eigen::Vector3d& W);

    double G(std::size_t n,
             const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& X,
             const Eigen::Vector3d& W,
             Eigen::Vector3d& PC,
             double& rsqr);


public:
    std::vector<Point> points;
    std::vector<Cluster> clusters;
    std::vector<std::vector<Cluster*> > row_clusters;
};

#endif // PILLAREXTRACTOR_H
