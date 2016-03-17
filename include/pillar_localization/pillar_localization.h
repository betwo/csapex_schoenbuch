#ifndef PILLAR_LOCALIZATION_H
#define PILLAR_LOCALIZATION_H


/// PROJECT
#include "ekf/ekf.h"
#include "pillar_localization/pillar_extractor.h"

/// SYSTEM
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class PillarLocalization
{
public:
    PillarLocalization();

    void applyMeasurement(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input);

    void applyOdometry(const nav_msgs::Odometry& odom);

    tf::StampedTransform getPose() const;

public:
    PillarExtractor pillar_extractor_;
    EKF ekf_;

    int init_step_;
    int init_steps_;
    bool init_;
    std::string fixed_frame_;

private:
    std::vector< std::vector<Pillar> > init_set_;
    ros::Time last_stamp_;
    tf::Pose last_pose_;

    tf::StampedTransform pose_;
};

#endif // PILLAR_LOCALIZATION_H
