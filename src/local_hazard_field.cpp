/// COMPONENT
#include "local_scrolling_map.h"

/// PROJECT
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/timer.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_transform/transform_message.h>
#include <csapex/utility/interlude.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace obstacle_detection;

namespace schoenbuch
{


namespace impl {

template <class PointT>
struct Impl;
}


class LocalHazardField : public Node
{
public:
    LocalHazardField()
        : cell_res_(0.0), res_(0.0), size_(0),
          prob_free_measurement_(0.0), prob_obstacle_measurement_(0.0), prob_obstacle_threshold_(0.0)
    {

    }

    void setup(NodeModifier& modifier) override
    {
        in_obstacle_ = modifier.addInput<PointCloudMessage>("Obstacle Cloud");
        in_transform_ = modifier.addInput<TransformMessage>("Transform map -> baselink");

        out_cloud_ = modifier.addOutput<PointCloudMessage>("Cloud");
        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("Marker");
    }

    void setupParameters(Parameterizable& params) override
    {
        params.addParameter(csapex::param::ParameterFactory::declareRange("range", 0.2, 10.0, 1.0, 0.1), range_);


        params.addParameter(csapex::param::ParameterFactory::declareRange("cell_resolution", 0.1, 10.0, 0.5, 0.1),
                            [&](csapex::param::Parameter* p) {
            cell_res_ = p->as<double>();
            update();
        });
        params.addParameter(csapex::param::ParameterFactory::declareRange("resolution", 0.01, 1.0, 0.05, 0.01),
                            [&](csapex::param::Parameter* p) {
            res_ = p->as<double>();
            update();
        });
        params.addParameter(csapex::param::ParameterFactory::declareRange("size", 0.01, 20.0, 5.0, 0.01),
                            [&](csapex::param::Parameter* p) {
            size_ = p->as<double>();
            update();
        });


        params.addParameter(csapex::param::ParameterFactory::declareRange("probability/threshold", 0.0, 1.0, 0.9, 0.01),
                            [&](csapex::param::Parameter* p) {
            prob_obstacle_threshold_ = p->as<double>();
            if(local_map_) {
                local_map_->prob_obstacle_threshold = prob_obstacle_threshold_;
            }
        });

        params.addParameter(csapex::param::ParameterFactory::declareRange("probability/measurement",
                                                                          csapex::param::ParameterDescription("Probability of a measurement beeing an obstacle"),
                                                                          0.01, 0.99, 0.7, 0.01),
                            [&](csapex::param::Parameter* p) {
            prob_obstacle_measurement_ = p->as<double>();
            if(local_map_) {
                local_map_->prob_obstacle_measurement = prob_obstacle_measurement_;
            }
        });
        params.addParameter(csapex::param::ParameterFactory::declareRange("probability/free",
                                                                          csapex::param::ParameterDescription("Probability update for free cells"),
                                                                          0.01, 0.99, 0.3, 0.01),
                            [&](csapex::param::Parameter* p) {
            prob_free_measurement_ = p->as<double>();
            if(local_map_) {
                local_map_->prob_free_measurement = prob_free_measurement_;
            }
        });

        params.addParameter(csapex::param::ParameterFactory::declareRange("probability/decay_rate", 0, 100, 0, 1),
                            decay_rate_);
        params.addParameter(csapex::param::ParameterFactory::declareRange("probability/decay", 0.01, 0.99, 0.2, 0.01),
                            decay_factor_);
        params.addParameter(csapex::param::ParameterFactory::declareRange("probability/decay_min_odds",
                                                                          0.01, 0.99, 0.3, 0.01),
                            [&](csapex::param::Parameter* p) {
            if(local_map_) {
                local_map_->min_prob_ = p->as<double>();
            }
        });


        next_decay_ = 0;
    }

    void process()
    {
        {
            INTERLUDE("map");

            is_floor_ = false;
            PointCloudMessage::ConstPtr msg_obstacle(msg::getMessage<PointCloudMessage>(in_obstacle_));
            boost::apply_visitor (PointCloudMessage::Dispatch<LocalHazardField>(this, msg_obstacle), msg_obstacle->value);
        }
        if(decay_rate_ > 0 && --next_decay_ <= 0){
            INTERLUDE("decay");
            local_map_->decay(decay_factor_);

            next_decay_ = decay_rate_;
        }
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        impl::Impl<PointT>::inputCloud(this, cloud);
    }

private:
    void update()
    {
        local_map_.reset(new LocalScrollingMap(size_, cell_res_, res_,
                                               prob_free_measurement_, prob_obstacle_measurement_, prob_obstacle_threshold_));
    }

public:

    template <typename PointT>
    void inputCloudImpl(schoenbuch::LocalHazardField* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        const pcl::PointCloud<PointT>& in = *cloud;
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = in.points;

        auto trafo = msg::getMessage<connection_types::TransformMessage>(instance->in_transform_);
        tf::Transform map2base = trafo->value;
        pos_ = map2base.getOrigin();

        if(local_map_) {
            local_map_->scroll(pos_);

            tf::Vector3 sensor_origin;//(0.08, 0.02, 0.4);
            for(const auto& point : points) {
                if(!std::isnan(point.x)) {
                    tf::Vector3 pt(point.x, point.y, point.z);
                    auto map_pt = map2base * pt;

                    local_map_->add(sensor_origin, map_pt, is_floor_);
                }
            }

            typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>& c = *cloud_ptr;

            tf::Transform base2map = map2base.inverse();

            local_map_->visitPoints([&](const tf::Vector3& pt_map) {
                tf::Vector3 pt = base2map * pt_map;

                PointT pcl_pt;
                pcl_pt.x = pt.x();
                pcl_pt.y = pt.y();
                pcl_pt.z = pt.z();
                c.push_back(pcl_pt);
            });

            auto message = std::make_shared<PointCloudMessage>(trafo->child_frame, cloud->header.stamp);
            message->value = cloud_ptr;
            msg::publish(out_cloud_, message);


            msg::publish<visualization_msgs::MarkerArray>(out_marker_, local_map_->visualize());
        }
    }

private:
    Input* in_obstacle_;
    Input* in_transform_;

    Output* out_cloud_;
    Output* out_marker_;

    tf::Vector3 pos_;

    double cell_res_;
    double res_;
    double size_;

    double prob_free_measurement_;
    double prob_obstacle_measurement_;
    double prob_obstacle_threshold_;

    int decay_rate_;
    int next_decay_;
    double decay_factor_;

    double range_;

    bool is_floor_;

    std::shared_ptr<LocalScrollingMap> local_map_;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(schoenbuch::LocalHazardField* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        instance->inputCloudImpl<PointT>(instance, cloud);

        //        throw std::runtime_error(std::string("point type '") + type2name(typeid(PointT)) + "' not supported");
    }
};

//template <>
//struct Impl<pcl::PointXYZL>
//{
//    static void inputCloud(sbc15::LocalHazardField* instance, typename pcl::PointCloud<pcl::PointXYZL>::ConstPtr cloud)
//    {
//        instance->inputCloudImpl<pcl::PointXYZL>(instance, cloud);
//    }
//};
//template <>
//struct Impl<pcl::PointXYZ>
//{
//    static void inputCloud(sbc15::LocalHazardField* instance, typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//    {
//        instance->inputCloudImpl(instance, cloud);
//    }
//};
}

}

CSAPEX_REGISTER_CLASS(schoenbuch::LocalHazardField, Node)


