/// HEADER
#include "pillar_localization/pillar_localization.h"

/// SYSTEM
#include <ros/ros.h>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace schoenbuch;

PillarLocalization::PillarLocalization()
    : init_step_(0), init_steps_(0), init_(false)
{

}

void PillarLocalization::reset()
{
    init_step_ = 0;
    init_ = false;

    init_set_.clear();

    ekf_.reset();
}

bool PillarLocalization::applyMeasurement(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input, bool only_absolute)
{
    std::vector<Pillar> pillars = pillar_extractor_.findPillars(input);

    if(!init_) {
        if(pillars.size() != 3) {
            ROS_WARN("cannot initialize, need exactly 3 pillar candiates");
            return false;
        }

        init_set_.push_back(pillars);
        ++init_step_;

        if(init_step_ <= init_steps_) {
            ROS_INFO_STREAM("initialization: " << (init_step_ / (double) init_steps_ * 100.0) << " %");
            return false;
        }

        // calculate mean distances
        std::size_t n = init_set_.size();
        double d[3];
        for(std::size_t k = 0; k < 3; ++k) {
            d[k] = 0;
        }

        for(std::size_t i = 0; i < n; ++i) {
            const std::vector<Pillar>& pillars = init_set_[i];

            Eigen::Vector3d pillar_a = pillars[0].centre;
            Eigen::Vector3d pillar_b = pillars[1].centre;
            Eigen::Vector3d pillar_c = pillars[2].centre;

            Eigen::Vector3d ab = pillar_b - pillar_a;
            Eigen::Vector3d ac = pillar_c - pillar_a;
            Eigen::Vector3d bc = pillar_c - pillar_b;

            std::vector<double> dists_ref;
            dists_ref.push_back(ac.norm());
            dists_ref.push_back(ab.norm());
            dists_ref.push_back(bc.norm());

            std::sort(dists_ref.begin(), dists_ref.end());
            for(std::size_t k = 0; k < 3; ++k) {
                d[k] += dists_ref[k];
            }

            ROS_WARN_STREAM("set " << i << ": " << dists_ref[0] << ", " << dists_ref[1] << ", " << dists_ref[2]);
        }

        for(std::size_t k = 0; k < 3; ++k) {
            d[k] /= n;
        }

        ROS_WARN_STREAM("mean: " << d[0] << ", " << d[1] << ", " << d[2]);

        //            int arg_min_k = 0;
        //            for(int k = 1; k < 3; ++k) {
        //                if(d[k] < d[arg_min_k]) {
        //                    arg_min_k = k;
        //                }
        //            }

        std::vector<std::pair<double, Pillar> > pillars_resorted;
        for(std::size_t k = 0; k < 3; ++k) {
            std::pair<double, Pillar> pair;
            pair.second = pillars[k];
            const Pillar& p1 = pillars[(k+1)%3];
            const Pillar& p2 = pillars[(k+2)%3];
            pair.first = (p1.centre - p2.centre).norm();

            pillars_resorted.push_back(pair);
        }
        std::sort(pillars_resorted.begin(), pillars_resorted.end());


        ROS_WARN_STREAM("sorted: " << pillars_resorted[0].first << ", " << pillars_resorted[1].first << ", " << pillars_resorted[2].first);

        // synthesize new pillars with the mean distances
        double alpha[3];
        for(std::size_t k = 0; k < 3; ++k) {
            double d1 = d[(k+1)%3];
            double d2 = d[(k+2)%3];
            alpha[k] = std::acos((d1*d1 + d2*d2 - d[k]*d[k]) / (2 * d1 * d2));
        }

        ROS_WARN_STREAM("angles: " << alpha[0] << ", " << alpha[1] << ", " << alpha[2] << "  |  " << (alpha[0] + alpha[1] + alpha[2]));


        std::vector<Pillar> pillars_synth;
        Pillar anchor;// = pillars_resorted[0].second;
        pillars_synth.push_back(anchor);

        Pillar lever = anchor;
        lever.centre += Eigen::Vector3d(d[2], 0, 0);
        pillars_synth.push_back(lever);


        Pillar last = anchor;
        double a = -alpha[0];
        last.centre += Eigen::Vector3d(std::cos(a) * d[1], std::sin(a) * d[1], 0);
        pillars_synth.push_back(last);

        ekf_.setPillars(pillars_synth);

        init_ = true;

        ROS_INFO("initialization done");
    }

    ROS_INFO_STREAM(pillars.size() << " pillar candidates");

    bool success = false;

    if(pillars.size() > 0) {
        if(only_absolute) {
            if(pillars.size() == 3) {
                success = ekf_.correctAbsolute(pillars, true);
            }
        } else {
            ekf_.correct(pillars);
            success = true;
        }

        updatePose(last_stamp_);

        last_stamp_ = ros::Time();
    }

    return success;
}

void PillarLocalization::updatePose(ros::Time current_stamp)
{
    tf::Vector3 origin(ekf_.mu(0), ekf_.mu(1), 0);
    tf::Quaternion q = tf::createQuaternionFromYaw(ekf_.mu(2));
    pose_ = tf::StampedTransform(tf::Transform(q, origin).inverse(),
                              current_stamp, "/velodyne", fixed_frame_);
}

bool PillarLocalization::fixPosition(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input)
{
    return applyMeasurement(input, true);
}

void PillarLocalization::applyOdometry(const nav_msgs::Odometry &odom)
{
    if(init_) {
        ros::Time current_stamp = odom.header.stamp;
        tf::Pose current_odom_pose;
        tf::poseMsgToTF(odom.pose.pose, current_odom_pose);

        if(last_stamp_ == ros::Time()) {
            last_stamp_ = current_stamp;
            last_pose_ = current_odom_pose;
            return;
        }

        ros::Duration dur = current_stamp - last_stamp_;
        last_stamp_ = current_stamp;

        tf::Pose delta_pose = last_pose_.inverse() * current_odom_pose;
        last_pose_ = current_odom_pose;

        Eigen::Vector3d delta(delta_pose.getOrigin().x(), delta_pose.getOrigin().y(),
                              tf::getYaw(delta_pose.getRotation()));

        double dt = dur.toSec();

        ekf_.predict(delta, odom.twist.twist.linear.x, odom.twist.twist.angular.z, dt);


        updatePose(current_stamp);
    }
}

tf::StampedTransform PillarLocalization::getPose() const
{
    return pose_;
}
