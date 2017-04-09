/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_transform/transform_message.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex_ros/tf_listener.h>

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

class VelodyneUndistortion : public Node
{
public:
    VelodyneUndistortion()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");

        out_ = modifier.addOutput<PointCloudMessage>("Undistorted Cloud");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareValue("scan/duration", 0.095), scan_duration_);
        params.addParameter(param::ParameterFactory::declareValue("scan/offset", 0.06), scan_offset_);

    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<VelodyneUndistortion>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr input)
    {
        const pcl::PointCloud<PointT>& cloud = *input;
        std::size_t cols = cloud.width;
        std::size_t rows = cloud.height;

        apex_assert_hard(cols > 1 && rows > 1);
        apex_assert_hard(cloud.isOrganized());

        LockedTFListener l = TFListener::getLocked();
        apex_assert(l.l);
        auto listener = l.l->tfl;
        apex_assert(listener);
        tf::TransformListener& tfl_ = *listener;

        ros::Time start_time, end_time;
        end_time.fromNSec(input->header.stamp * 1e3);
        end_time += ros::Duration(scan_offset_);
        start_time = end_time - ros::Duration(scan_duration_);

        std::string fixed_frame = "/odom";

        if(!tfl_.waitForTransform(
                    fixed_frame,
                    input->header.frame_id,
                    start_time,
                    ros::Duration(std::max(0.0, scan_offset_) + 0.1)))
        {
            aerr << "cannot transform start cloud at time " << start_time << ", now is " << ros::Time::now()<< std::endl;
            return;
        }

        if(!tfl_.waitForTransform(
                    fixed_frame,
                    input->header.frame_id,
                    end_time,
                    ros::Duration(std::max(0.0, scan_offset_) + 0.1)))
        {
            aerr << "cannot transform end cloud at time " << end_time << ", now is " << ros::Time::now() << std::endl;
            return;
        }

        tf::StampedTransform fixed_T_end;
        tfl_.lookupTransform(fixed_frame, input->header.frame_id, end_time, fixed_T_end);

        tf::Transform end_T_fixed = fixed_T_end.inverse();

        typename pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>);
        res->header = input->header;
        res->header.stamp = end_time.toNSec() * 1e-3;
        res->width = input->width;
        res->height = input->height;
        res->points = input->points;

        ros::Time stamp = start_time;
        ros::Duration delta = (end_time - start_time) * (1.0 / cols);

        for(std::size_t c = 0; c < cols; ++c, stamp += delta) {
            tf::StampedTransform fixed_T_current;
            tfl_.lookupTransform(fixed_frame, input->header.frame_id, stamp, fixed_T_current);
            tf::Transform end_T_current = end_T_fixed * fixed_T_current;

            for(std::size_t r = 0; r < rows; ++r) {
                PointT& pt = res->at(c, r);
                tf::Vector3 pt_current(pt.x, pt.y, pt.z);
                tf::Vector3 pt_end = end_T_current * pt_current;
                pt.x = pt_end.x();
                pt.y = pt_end.y();
                pt.z = pt_end.z();
            }
        }

        PointCloudMessage::Ptr output_msg(new PointCloudMessage(cloud.header.frame_id, cloud.header.stamp));
        output_msg->value = res;

        msg::publish(out_, output_msg);
    }

private:
    Input* in_;
    Output* out_;

    double scan_duration_;
    double scan_offset_;
};


}

CSAPEX_REGISTER_CLASS(csapex::VelodyneUndistortion, csapex::Node)

