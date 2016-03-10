/// PROJECT
#include "ekf.h"
#include "pillar_extractor.h"

/// SYSTEM
#include <ros/ros.h>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

class PillarLocalizationNode
{
public:
    PillarLocalizationNode()
        : pnh_("~"), init_(false)
    {
        sub_velodyne_ = nh_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, boost::bind(&PillarLocalizationNode::cloudCallback, this, _1));
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1000, boost::bind(&PillarLocalizationNode::odomCallback, this, _1));

        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10, false);
        pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, false);

        fixed_frame_ = pnh_.param("fixed_frame", std::string("/pillars"));
     
        ekf_.dist_threshold_ = pnh_.param("threshold", 0.25);

        pillar_extractor_.radius_ = pnh_.param("radius", 0.055);
        pillar_extractor_.min_pts_ = pnh_.param("min_pts", 5);
        pillar_extractor_.max_range_  = pnh_.param("max_range", 15.0);
        pillar_extractor_.min_intensity_ = pnh_.param("min_intensity", 150);
        pillar_extractor_.min_cluster_size_= pnh_.param("min_cluster_size", 5);
        pillar_extractor_.cluster_tolerance_ = pnh_.param("cluster_tolerance", 0.7);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        std::vector<Pillar> pillars = pillar_extractor_.findPillars(input);

        if(!init_) {
            if(pillars.size() != 3) {
                ROS_WARN("cannot initialize, need exactly 3 pillar candiates");
                return;
            }

            ekf_.setPillars(pillars);

            init_ = true;

            ROS_INFO("initialization done");
        }

        ROS_INFO_STREAM(pillars.size() << " pillar candidates");

        if(pillars.size() > 0) {
//            ekf_.correct(pillars);
            ekf_.correctAbsolute(pillars);

            last_stamp_ = ros::Time();
        }
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& odom)
    {
        if(init_) {
            ros::Time current_stamp = odom->header.stamp;
            tf::Pose current_odom_pose;
            tf::poseMsgToTF(odom->pose.pose, current_odom_pose);

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

            ekf_.predict(delta, odom->twist.twist.linear.x, odom->twist.twist.angular.z, dt);


            tf::Vector3 origin(ekf_.mu(0), ekf_.mu(1), 0);
            tf::Quaternion q = tf::createQuaternionFromYaw(ekf_.mu(2));
            tf::StampedTransform pose(tf::Transform(q, origin).inverse(),
                                      current_stamp, "/velodyne", fixed_frame_);

            tfb_.sendTransform(pose);


            visualization_msgs::Marker marker;
            marker.header.frame_id = fixed_frame_;
            marker.header.stamp = current_stamp;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.ns = "pillar_ekf";
            marker.pose.position.x = ekf_.mu(0);
            marker.pose.position.y = ekf_.mu(1);
            //            tf::quaternionTFToMsg(pose.getRotation(), marker.pose.orientation);
            marker.pose.orientation.w = 1.0;
            marker.color.r = 1.0;
            marker.color.a = 0.7;
            marker.scale.x = std::sqrt(ekf_.P(0,0));
            marker.scale.y = std::sqrt(ekf_.P(1,1));
            marker.scale.z = std::sqrt(ekf_.P(2,2));

            pub_marker_.publish(marker);



            visualization_msgs::MarkerArray markers;

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.ns = "pillar";
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.scale.x = 2* pillar_extractor_.radius_;
            marker.scale.y = 2* pillar_extractor_.radius_;
            marker.scale.z = 1.0;

            for(std::size_t i = 0; i < ekf_.pillars_.size(); ++i) {
                Eigen::Vector3d pos = ekf_.pillars_[i].centre;
                marker.pose.position.x = pos(0);
                marker.pose.position.y = pos(1);

                markers.markers.push_back(marker);

                marker.id++;
            }

            marker.color.b = 1.0;

            for(std::size_t i = 0; i < ekf_.meas_pillars_.size(); ++i) {
                Eigen::Vector3d pos = ekf_.meas_pillars_[i].centre;
                marker.pose.position.x = pos(0);
                marker.pose.position.y = pos(1);

                markers.markers.push_back(marker);

                marker.id++;
            }

            pub_marker_array_.publish(markers);
        }
    }

private:

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_velodyne_;

    ros::Publisher pub_marker_;
    ros::Publisher pub_marker_array_;

    tf::TransformBroadcaster tfb_;

    PillarExtractor pillar_extractor_;
    EKF ekf_;

    bool init_;

    std::string fixed_frame_;

    ros::Time last_stamp_;
    tf::Pose last_pose_;
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

    ros::Rate rate(60);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

