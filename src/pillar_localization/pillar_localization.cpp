/// HEADER
#include <pillar_localization/pillar_localization.h>

/// SYSTEM
#include <ros/ros.h>
#include <ros/time.h>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace schoenbuch;

PillarLocalization::PillarLocalization()
    : init_step_(0), init_steps_(0), init_(false), last_stamp_(0)
{
    scan_duration_ = ros::Duration(1.0 / 10.0);
    scan_offset_ = ros::Duration(0.0);
}

void PillarLocalization::reset()
{
    init_step_ = 0;
    init_ = false;

    init_set_.clear();

    ekf_.reset();
}

pcl::PointCloud<pcl::PointXYZI>::ConstPtr PillarLocalization::undistort(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input)
{
    ros::Time start_time, end_time;
    end_time.fromNSec(input->header.stamp * 1e3);
    end_time += scan_offset_;
    start_time = end_time - scan_duration_;

    std::string fixed_frame = "/odom";

    while(!tfl_.waitForTransform(
              fixed_frame,
              input->header.frame_id,
              start_time,
              ros::Duration(0.3)))
    {
        ROS_WARN_STREAM("waiting for start transform from " << input->header.frame_id << " to odom at time " << start_time);
        if(start_time < ros::Time::now() - ros::Duration(1.0) && !tfl_.canTransform(input->header.frame_id, fixed_frame, start_time)) {
            ROS_WARN_STREAM("cannot transform cloud at time " << end_time << ", now is " << ros::Time::now());
            return input;
        }
    }

    while(!tfl_.waitForTransform(
              fixed_frame,
              input->header.frame_id,
              end_time,
              ros::Duration(0.3)))
    {
        ROS_WARN_STREAM("waiting for end transform from " << input->header.frame_id << " to odom at time " << end_time);
        if(end_time < ros::Time::now() - ros::Duration(1.0) && !tfl_.canTransform(input->header.frame_id, fixed_frame, end_time)) {
            ROS_WARN_STREAM("cannot transform cloud at time " << end_time << ", now is " << ros::Time::now());
            return input;
        }
    }

    ros::WallTime profile_start = ros::WallTime::now();

    tf::StampedTransform fixed_T_end;
    tfl_.lookupTransform(fixed_frame, input->header.frame_id, end_time, fixed_T_end);

    tf::Transform end_T_fixed = fixed_T_end.inverse();

    pcl::PointCloud<pcl::PointXYZI>::Ptr res(new pcl::PointCloud<pcl::PointXYZI>);
    res->header = input->header;
    res->header.stamp = end_time.toNSec() * 1e-3;
    res->width = input->width;
    res->height = input->height;
    res->points = input->points;

    std::size_t cols = res->width;
    std::size_t rows = res->height;
    ros::Time stamp = start_time;
    ros::Duration delta = (end_time - start_time) * (1.0 / cols);

    for(std::size_t c = 0; c < cols; ++c, stamp += delta) {
        tf::StampedTransform fixed_T_current;
        tfl_.lookupTransform(fixed_frame, input->header.frame_id, stamp, fixed_T_current);
        tf::Transform end_T_current = end_T_fixed * fixed_T_current;

        for(std::size_t r = 0; r < rows; ++r) {
            pcl::PointXYZI& pt = res->at(c, r);
            tf::Vector3 pt_current(pt.x, pt.y, pt.z);
            tf::Vector3 pt_end = end_T_current * pt_current;
            pt.x = pt_end.x();
            pt.y = pt_end.y();
            pt.z = pt_end.z();
        }
    }


    ros::WallTime profile_end = ros::WallTime::now();

    ros::WallDuration profile = profile_end - profile_start;
    ROS_WARN_STREAM("transforming the point cloud took " << (profile * 1e3) << "ms");

    return res;
}

pcl::PointCloud<pcl::PointXYZI>::ConstPtr PillarLocalization::getUndistortedCloud() const
{
    return undistorted_cloud_;
}

bool PillarLocalization::applyMeasurement(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input, bool only_absolute)
{
    undistorted_cloud_ = undistort(input);

    std::vector<Pillar> pillars = pillar_extractor_.findPillars(undistorted_cloud_);

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

        if(init_steps_ <= 0) {
            ekf_.setPillars(pillars);

            init_ = true;

        } else {
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
        }

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
    ros::Time current_stamp = odom.header.stamp;
    tf::Pose current_odom_pose;
    tf::poseMsgToTF(odom.pose.pose, current_odom_pose);

    if(last_stamp_ == ros::Time(0)) {
        ROS_WARN_STREAM("set last pose, current is " << last_stamp_);
        last_stamp_ = current_stamp;
        last_pose_ = current_odom_pose;
        return;
    }

    ros::Duration dur = current_stamp - last_stamp_;
    last_stamp_ = current_stamp;
    double dt = dur.toSec();

    tf::Pose delta_pose = last_pose_.inverse() * current_odom_pose;

    Eigen::Vector3d delta(delta_pose.getOrigin().x(), delta_pose.getOrigin().y(),
                          tf::getYaw(delta_pose.getRotation()));


    if(ekf_.predict(delta, odom.twist.twist.linear.x, odom.twist.twist.angular.z, dt)) {
        last_pose_ = current_odom_pose;
    }

    updatePose(current_stamp);
}

tf::StampedTransform PillarLocalization::getPose() const
{
    return pose_;
}
