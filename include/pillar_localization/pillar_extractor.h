#ifndef PILLAREXTRACTOR_H
#define PILLAREXTRACTOR_H

/// PROJECT
#include "pillar.h"

/// SYSTEM
#include <tf/tf.h>
#include <memory>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PillarExtractorImpl;

class PillarExtractor
{
public:
    PillarExtractor();
    ~PillarExtractor();

    std::vector<Pillar> findPillars(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input);

public:
    int min_pts_;
    int min_intensity_;
    double radius_;
    double max_range_;
    int min_cluster_size_;
    double cluster_tolerance_;

private:
    PillarExtractorImpl* pimpl;
};

#endif // PILLAREXTRACTOR_H
