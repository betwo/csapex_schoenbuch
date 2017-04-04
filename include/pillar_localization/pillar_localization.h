#ifndef PILLAR_LOCALIZATION_H
#define PILLAR_LOCALIZATION_H


/// PROJECT
#include "ekf/ekf.h"
#include "pillar_localization/pillar_extractor.h"

/// SYSTEM
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace schoenbuch
{

class PillarLocalization
{
public:
    PillarLocalization();

    bool fixPosition(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input);

    bool applyMeasurement(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input, bool only_absolute = false);
    void applyOdometry(const nav_msgs::Odometry& odom);

    void reset();

    tf::StampedTransform getPose() const;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr getUndistortedCloud() const;

public:
    PillarExtractor pillar_extractor_;
    EKF ekf_;

    int init_step_;
    int init_steps_;
    bool init_;

    bool undistort_;

    std::string fixed_frame_;
    ros::Duration scan_duration_;
    ros::Duration scan_offset_;

private:
    void updatePose(ros::Time current_stamp);
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr undistort(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input);


private:
    std::vector< std::vector<Pillar> > init_set_;
    ros::Time last_stamp_;
    tf::Pose last_pose_;

    std::shared_ptr<tf::TransformListener> tfl_;
    tf::StampedTransform pose_;


    pcl::PointCloud<pcl::PointXYZI>::ConstPtr undistorted_cloud_;
};

}

#endif // PILLAR_LOCALIZATION_H
