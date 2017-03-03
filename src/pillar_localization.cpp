/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_transform/transform_message.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>

/// COMPONENT
#include "pillar_localization/pillar_localization.h"

/// SYSTEM
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;
}


class PillarLocalization : public Node
{
public:
    PillarLocalization()
        : has_start_pose_(false)
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");
        input_odom_ = modifier.addOptionalInput<nav_msgs::Odometry>("Odometry");

        out_ = modifier.addOutput<TransformMessage>("Pose");
        out_rel_ = modifier.addOutput<TransformMessage>("Pose (relative)");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("threshold", 0.0, 1.0, 0.25, 0.001),
                            localization_.ekf_.dist_threshold_);

        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance on ring", 0.0, 0.1, 0.05, 0.001),
                            localization_.pillar_extractor_.cluster_distance_ring_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance vertically", 0.0, 0.1, 0.05, 0.001),
                            localization_.pillar_extractor_.cluster_distance_vertical_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance euclidean", 0.0, 1.0, 0.5, 0.01),
                            localization_.pillar_extractor_.cluster_distance_euclidean_);

        params.addParameter(param::ParameterFactory::declareRange("clustering/min cluster size", 0, 1024, 32, 1),
                            localization_.pillar_extractor_.cluster_min_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster size", 0, 1024, 32, 1),
                            localization_.pillar_extractor_.cluster_max_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster diameter", 0.0, 5.0, 0.2, 0.01),
                            localization_.pillar_extractor_.cluster_max_diameter_);

        params.addParameter(param::ParameterFactory::declareRange("pillar/min intensity", 0.0, 1024.0, 120.0, 0.1),
                            localization_.pillar_extractor_.pillar_min_intensity_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/min points", 0, 512, 32, 1),
                            localization_.pillar_extractor_.pillar_min_points_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/radius", 0.001, 1.0, 0.055, 0.001),
                            localization_.pillar_extractor_.pillar_radius_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/radius threshold", 0.000, 1.0, 0.055, 0.001),
                            localization_.pillar_extractor_.pillar_radius_fuzzy_);

        params.addParameter(param::ParameterFactory::declareBool("undistortion", true), undistort_);

        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*) {
            reset();
        });

        localization_.fixed_frame_ = "/pillars";

        localization_.init_steps_ = 10;
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<PillarLocalization>(this, msg), msg->value);
    }

    void reset()
    {
        localization_.reset();

        has_start_pose_ = false;
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        impl::Impl<PointT>::inputCloud(this, cloud);
    }


    void inputCloudImpl(typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        bool success = false;
        if(msg::hasMessage(input_odom_)) {
            auto odom = msg::getMessage<nav_msgs::Odometry>(input_odom_);
            localization_.applyOdometry(*odom);
            localization_.applyMeasurement(cloud, undistort_);

            success = true;

        } else {
            success = localization_.fixPosition(cloud);
        }

        if(!success) {
            return;
        }

        tf::Transform pose = localization_.getPose();

        TransformMessage::Ptr result = std::make_shared<TransformMessage>("/pillars", "/base_link");
        result->value = pose;
        result->stamp_micro_seconds = cloud->header.stamp * 1e3;
        result->frame_id = "/pillars";

        msg::publish(out_, result);

        if(!has_start_pose_) {
            start_pose_ = pose;
            has_start_pose_ = true;
        }

        TransformMessage::Ptr result_rel = std::make_shared<TransformMessage>("/pillars", "/base_link");
        *result_rel = *result;

        result_rel->value = pose * start_pose_.inverse();

        msg::publish(out_rel_, result_rel);
    }

private:
    Input* in_;
    Input* input_odom_;

    Output* out_;
    Output* out_rel_;

    bool has_start_pose_;
    tf::Pose start_pose_;

    bool undistort_;

    schoenbuch::PillarLocalization localization_;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(PillarLocalization* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        throw std::runtime_error(std::string("point type '") + type2name(typeid(PointT)) + "' not supported");
    }
};

template <>
struct Impl<pcl::PointXYZI>
{
    static void inputCloud(PillarLocalization* instance, typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        instance->inputCloudImpl(cloud);
    }
};

}

}

CSAPEX_REGISTER_CLASS(csapex::PillarLocalization, csapex::Node)

