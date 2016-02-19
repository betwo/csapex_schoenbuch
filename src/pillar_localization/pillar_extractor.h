#ifndef PILLAREXTRACTOR_H
#define PILLAREXTRACTOR_H

/// SYSTEM
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <Eigen/Core>

class PillarExtractorImpl;

class PillarExtractor
{
public:
    PillarExtractor();
    ~PillarExtractor();

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > findPillars(const sensor_msgs::PointCloud2::ConstPtr& input);

public:
    int min_pts_;
    int min_intensity_;
    double radius_;
    int min_cluster_size_;
    double cluster_tolerance_;

private:
    PillarExtractorImpl* pimpl;
};

#endif // PILLAREXTRACTOR_H
