/// PROJECT
#include "pillar_localization/pillar_localization.h"

/// SYSTEM
#include <ros/ros.h>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>

class PillarLocalizationNode
{
public:
    PillarLocalizationNode()
        : pnh_("~")
    {
        sub_velodyne_ = nh_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, boost::bind(&PillarLocalizationNode::cloudCallback, this, _1));
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1000, boost::bind(&PillarLocalizationNode::odomCallback, this, _1));

        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10, false);
        pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, false);
        pub_undistorted_ = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_points/undistored", 10, false);

        localization_.fixed_frame_ = pnh_.param("fixed_frame", std::string("/pillars"));

        localization_.init_steps_ = pnh_.param("init_steps", 0);

        localization_.ekf_.dist_threshold_ = pnh_.param("threshold", 0.25);

        localization_.scan_duration_ = ros::Duration(pnh_.param("scan_duration", 1.0 / 10.0));
        localization_.scan_offset_ = ros::Duration(pnh_.param("scan_offset", 0.0));

        localization_.pillar_extractor_.pillar_radius_ = pnh_.param("radius", 0.065);
        localization_.pillar_extractor_.pillar_radius_fuzzy_ = pnh_.param("radius_threshold", 0.04);
        localization_.pillar_extractor_.pillar_min_points_ = pnh_.param("min_pts", 4);
        localization_.pillar_extractor_.cluster_max_diameter_  = pnh_.param("cluster_max", 0.0);
        localization_.pillar_extractor_.pillar_min_intensity_ = pnh_.param("min_intensity", 140);
        localization_.pillar_extractor_.cluster_min_size_= pnh_.param("min_cluster_size", 2);
        localization_.pillar_extractor_.cluster_max_size_= pnh_.param("max_cluster_size", 37);
        localization_.pillar_extractor_.cluster_distance_ring_ = pnh_.param("cluster_tolerance_ring", 0.043);
        localization_.pillar_extractor_.cluster_distance_vertical_ = pnh_.param("cluster_tolerance_vertical", 0.026);
        localization_.pillar_extractor_.cluster_distance_euclidean_ = pnh_.param("cluster_tolerance_euclidean", 0.5);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(pcl_pc2,*full_cloud);

        localization_.applyMeasurement(full_cloud, true);

        pcl::PointCloud<pcl::PointXYZI>::ConstPtr undistorted = localization_.getUndistortedCloud();

        pcl::PCLPointCloud2 pcl_pc2_out;
        pcl::toPCLPointCloud2(*undistorted, pcl_pc2_out);
        sensor_msgs::PointCloud2::Ptr undistorted_ros(new sensor_msgs::PointCloud2);
        pcl_conversions::fromPCL(pcl_pc2_out, *undistorted_ros);

        pub_undistorted_.publish(undistorted_ros);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& odom)
    {
        localization_.applyOdometry(*odom);

        tf::StampedTransform pose = localization_.getPose();

        tfb_.sendTransform(pose);


        visualization_msgs::Marker marker;
        marker.header.frame_id = localization_.fixed_frame_;
        marker.header.stamp = pose.stamp_;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.ns = "pillar_ekf";
        marker.pose.position.x = localization_.ekf_.mu(0);
        marker.pose.position.y = localization_.ekf_.mu(1);
        //            tf::quaternionTFToMsg(pose.getRotation(), marker.pose.orientation);
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0;
        marker.color.a = 0.7;
        marker.scale.x = std::sqrt(localization_.ekf_.P(0,0));
        marker.scale.y = std::sqrt(localization_.ekf_.P(1,1));
        marker.scale.z = std::sqrt(localization_.ekf_.P(2,2));

        pub_marker_.publish(marker);



        visualization_msgs::MarkerArray markers;

        // MAP
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "pillar";
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.scale.x = 2* localization_.pillar_extractor_.pillar_radius_;
        marker.scale.y = 2* localization_.pillar_extractor_.pillar_radius_;
        marker.scale.z = 1.0;

        for(std::size_t i = 0; i < localization_.ekf_.pillars_.size(); ++i) {
            Eigen::Vector3d pos = localization_.ekf_.pillars_[i].centre;
            marker.pose.position.x = pos(0);
            marker.pose.position.y = pos(1);

            markers.markers.push_back(marker);

            marker.id++;
        }

        // MEASUREMENT
        marker.scale.x = localization_.pillar_extractor_.pillar_radius_;
        marker.scale.y = localization_.pillar_extractor_.pillar_radius_;
        marker.scale.z = 2.0;

        marker.color.b = 1.0;

        for(std::size_t i = 0; i < 3; ++i) {
            marker.header.frame_id = localization_.fixed_frame_;
            if(i >= localization_.ekf_.meas_pillars_.size()) {
                marker.action = visualization_msgs::Marker::DELETE;

            } else {
                marker.action = visualization_msgs::Marker::ADD;
                Eigen::Vector3d pos = localization_.ekf_.meas_pillars_[i].centre;
                marker.pose.position.x = pos(0);
                marker.pose.position.y = pos(1);
            }


            markers.markers.push_back(marker);
            marker.id++;
        }

        pub_marker_array_.publish(markers);
    }

private:
    schoenbuch::PillarLocalization localization_;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_velodyne_;

    ros::Publisher pub_marker_;
    ros::Publisher pub_marker_array_;
    ros::Publisher pub_undistorted_;

    tf::TransformBroadcaster tfb_;
};

void siginthandler(int)
{
    std::cerr << "User pressed Ctrl + C" << std::endl;
    raise(SIGTERM);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pillar_localization_node", ros::init_options::NoSigintHandler);

    signal(SIGINT, siginthandler);

    PillarLocalizationNode node;

    ros::Rate rate(120);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

