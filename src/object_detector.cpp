/// COMPONENT
#include "data/point.h"
#include "data/cluster.h"

#include "pillar_localization/pillar_extractor.h"

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
#include <csapex/view/utility/color.hpp>
#include <csapex/utility/timer.h>
#include <csapex/utility/interlude.hpp>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}

class ObjectDetector : public Node
{
public:
    ObjectDetector()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");

        out_ = modifier.addOutput<PointCloudMessage>("Clustered Cloud");
        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("Markers");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance on ring", 0.0, 0.1, 0.05, 0.001),
                            extractor_.cluster_distance_ring_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance vertically", 0.0, 0.1, 0.05, 0.001),
                            extractor_.cluster_distance_vertical_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/min cluster size", 0, 1024, 32, 1),
                            extractor_.cluster_min_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster size", 0, 1024, 32, 1),
                            extractor_.cluster_max_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster diameter", 0.0, 5.0, 0.2, 0.01),
                            extractor_.cluster_max_diameter_);

        params.addParameter(param::ParameterFactory::declareRange("pillar/min intensity", 0.0, 1024.0, 120.0, 0.1),
                            extractor_.pillar_min_intensity_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/min points", 0, 512, 32, 1),
                            extractor_.pillar_min_points_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/radius", 0.001, 1.0, 0.055, 0.001),
                            extractor_.pillar_radius_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/radius threshold", 0.000, 1.0, 0.055, 0.001),
                            extractor_.pillar_radius_fuzzy_);

    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<ObjectDetector>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        impl::Impl<PointT>::inputCloud(this, cloud);
    }

    Eigen::Vector3d convert(const pcl::PointXYZI& rhs) {
        return Eigen::Vector3d(rhs.x, rhs.y, rhs.z);
    }

    void inputCloudImpl(typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<pcl::PointXYZI>& cloud = *cloud_ptr;
        int cols = cloud.width;
        int rows = cloud.height;

        apex_assert_hard(cols > 1 && rows > 1);
        apex_assert_hard(cloud.isOrganized());

        std::vector<Pillar> pillars = extractor_.findPillars(cloud_ptr);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr labeled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>& labeled_cloud = *labeled_cloud_ptr;

        labeled_cloud.header = cloud.header;
        labeled_cloud.width = cloud.width;
        labeled_cloud.height = cloud.height;
        labeled_cloud.resize(cloud.size());

        for(int col = 0; col < cols; ++col) {
            for(int row = 0; row < rows; ++row) {
                const Point& pt = extractor_.points.at(row * cols + col);
                pcl::PointXYZRGB& pt_out = labeled_cloud.at(col, row);

                pt_out.x = pt.x;
                pt_out.y = pt.y;
                pt_out.z = pt.z;

                if(pt.cluster) {
                    double r = 0, g = 0, b = 0;
                    color::fromCount(pt.cluster->id, r,g,b);
                    pt_out.r = r;
                    pt_out.g = g;
                    pt_out.b = b;
                }
            }
        }

        PointCloudMessage::Ptr output_msg(new PointCloudMessage(cloud.header.frame_id, cloud.header.stamp));
        output_msg->value = labeled_cloud_ptr;

        msg::publish(out_, output_msg);


        if(msg::isConnected(out_marker_)) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = cloud.header.frame_id;
            marker.header.stamp.fromNSec(cloud.header.stamp * 1e3);
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.ns = "pillars";
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            marker.color.a = 0.7;

            auto message = std::make_shared<visualization_msgs::MarkerArray>();
            visualization_msgs::MarkerArray& marker_array = *message;

            int id = 0;
            for(Pillar& p : pillars) {
                marker.pose.position.x = p.centre(0);
                marker.pose.position.y = p.centre(1);
                marker.pose.position.z = p.centre(2);

                tf::Matrix3x3 base_T;
                tf::Vector3 z(p.up(0), p.up(1), p.up(2));
                tf::Vector3 x(1, 0, 0);
                tf::Vector3 y = z.cross(x);
                base_T[0] = x;
                base_T[1] = y;
                base_T[2] = z;

                //tf::Quaternion base(W(0),W(1),W(2),0);
                tf::Quaternion base;
                base_T.getRotation(base);
                tf::quaternionTFToMsg(base, marker.pose.orientation);

                marker.scale.x = p.measured_radius * 2;
                marker.scale.y = p.measured_radius * 2;
                marker.scale.z = 2.0;

                marker.id = id++;
                marker_array.markers.push_back(marker);
            }

            msg::publish(out_marker_, message);
        }
    }

private:
    Input* in_;

    Output* out_;
    Output* out_marker_;

    PillarExtractor extractor_;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(ObjectDetector* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        throw std::runtime_error(std::string("point type '") + type2name(typeid(PointT)) + "' not supported");
    }
};

template <>
struct Impl<pcl::PointXYZI>
{
    static void inputCloud(ObjectDetector* instance, typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        instance->inputCloudImpl(cloud);
    }
};

}

}

CSAPEX_REGISTER_CLASS(csapex::ObjectDetector, csapex::Node)

