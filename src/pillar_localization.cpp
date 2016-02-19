/// COMPONENT
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
        : init_(false), has_start_pose_(false)
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");
        input_indices_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("clusters");

        out_ = modifier.addOutput<TransformMessage>("Pose");
        out_rel_ = modifier.addOutput<TransformMessage>("Pose (relative)");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("radius", 0.01, 1.0, 0.20, 0.001), radius_);
        params.addParameter(param::ParameterFactory::declareRange("threshold", 0.0, 1.0, 0.10, 0.001), threshold_);
        params.addParameter(param::ParameterFactory::declareRange("min pts", 1, 100, 4, 1), min_pts_);

        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*) {
            reset();
        });
        params.addParameter(param::ParameterFactory::declareRange("distance/1", 0.01, 20.0, 0.05, 0.001), dist_1_);
        params.addParameter(param::ParameterFactory::declareRange("distance/2", 0.01, 20.0, 0.05, 0.001), dist_2_);
        params.addParameter(param::ParameterFactory::declareRange("distance/3", 0.01, 20.0, 0.05, 0.001), dist_3_);
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<PillarLocalization>(this, msg), msg->value);
    }

    void reset()
    {
        init_ = false;
        has_start_pose_ = false;

        last_stamp_ = 0;
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        impl::Impl<PointT>::inputCloud(this, cloud);
    }


    void inputCloudImpl(typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        auto current_stamp = cloud->header.stamp;
        if(last_stamp_ == 0) {
            last_stamp_ = current_stamp;
            return;
        }

        int dt_msec = current_stamp - last_stamp_;
        last_stamp_ = current_stamp;

        double dt = dt_msec / 1e6;

        std::vector<pcl::PointIndices> clusters = *msg::getMessage<GenericVectorMessage, pcl::PointIndices>(input_indices_);

        std::vector<std::pair<int, tf::Vector3>> pillar_candidates;

        for(const pcl::PointIndices& indices_msg : clusters) {
            if((int) indices_msg.indices.size() < min_pts_) {
                continue;
            }

            double angle = 0;
            double dist_sqr = std::numeric_limits<double>::infinity();

            for(int i : indices_msg.indices) {
                const pcl::PointXYZI& pt = cloud->points[i];

                double d_sqr = (pt.x * pt.x + pt.y * pt.y/* + pt.z * pt.z*/);
                if(d_sqr < dist_sqr) {
                    dist_sqr = d_sqr;
                    angle = std::atan2(pt.y, pt.x);
                }

            }

            if(std::isfinite(dist_sqr)) {
                double r = std::sqrt(dist_sqr) + radius_;

                tf::Vector3 centre(std::cos(angle) * r,
                                   std::sin(angle) * r,
                                   0.0);

                pillar_candidates.push_back(std::make_pair(indices_msg.indices.size(), centre));
            }
        }

        std::sort(pillar_candidates.begin(), pillar_candidates.end(), std::greater<std::pair<int,tf::Vector3>>());


        if(!init_) {
            if(pillar_candidates.size() != 3) {
                awarn << "cannot initialize, need exactly 3 pillar candiates" << std::endl;
                return;
            }

            pillar_a = pillar_candidates[0].second;
            pillar_b = pillar_candidates[1].second;
            pillar_c = pillar_candidates[2].second;

            tf::Vector3 ab = pillar_b - pillar_a;
            tf::Vector3 ac = pillar_c - pillar_a;
            tf::Vector3 bc = pillar_c - pillar_b;

            dist_1_ = (ac).length();
            dist_2_ = (ab).length();
            dist_3_ = (bc).length();

            setParameter("distance/1", dist_1_);
            setParameter("distance/2", dist_2_);
            setParameter("distance/3", dist_3_);

            init_ = true;

            ainfo << "initialization done" << std::endl;
        }

        if(pillar_candidates.size() >= 3) {
            tf::Vector3 a = pillar_candidates[0].second;
            tf::Vector3 b = pillar_candidates[1].second;
            tf::Vector3 c = pillar_candidates[2].second;

            tf::Vector3 ab = b - a;
            tf::Vector3 ac = c - a;
            tf::Vector3 bc = c - b;

            double d_ac = (ac).length();
            double d_ab = (ab).length();
            double d_bc = (bc).length();

            double mind = std::min(d_ac, std::min(d_ab, d_bc));

            tf::Vector3 anchor;
            tf::Vector3 lever;
            if(d_ab == mind) {
                anchor = c;
                lever = (d_ac < d_bc) ? a : b;
            } else if(d_ac == mind) {
                anchor = b;
                lever = (d_ab < d_bc) ? a : c;
            } else {
                anchor = a;
                lever = (d_ab < d_ac) ? b : c;
            }

            std::vector<double> dists_meas { d_ac, d_ab, d_bc };
            std::vector<double> dists_ref {dist_1_, dist_2_, dist_3_ };

            std::sort(dists_meas.begin(), dists_meas.end());
            std::sort(dists_ref.begin(), dists_ref.end());

            double max_delta = 0;
            for(std::size_t i = 0; i < 3; ++i) {
                double delta = std::abs(dists_meas[i] - dists_ref[i]);
                if(delta > max_delta) {
                    max_delta = delta;
                }
            }

            if(max_delta < threshold_) {
                tf::Vector3 base_line = lever - anchor;
                double yaw_correction = std::atan2(base_line.y(), base_line.x());

                tf::Vector3 origin(anchor.x(), anchor.y(), 0);
                tf::Quaternion q = tf::createQuaternionFromYaw(yaw_correction);
                tf::Transform pose(q, origin);// tf::quatRotate(q, origin));

                //correct(a.length(), b.length(), c.length());

                TransformMessage::Ptr result = std::make_shared<TransformMessage>("/pillars", "/base_link");
                result->value = pose;

                msg::publish(out_, result);

                if(!has_start_pose_) {
                    start_pose_ = pose;
                    has_start_pose_ = true;
                }

                TransformMessage::Ptr result_rel = std::make_shared<TransformMessage>("/pillars", "/base_link");
                result_rel->value = pose * start_pose_.inverse();

                msg::publish(out_rel_, result_rel);

            }
        }
    }

private:

private:
    Input* in_;
    Input* input_indices_;

    Output* out_;
    Output* out_rel_;

    bool init_;

    int min_pts_;

    double radius_;
    double threshold_;


    tf::Vector3 pillar_a;
    tf::Vector3 pillar_b;
    tf::Vector3 pillar_c;

    double dist_1_;
    double dist_2_;
    double dist_3_;

    uint64_t last_stamp_;

    bool has_start_pose_;
    tf::Transform start_pose_;
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

