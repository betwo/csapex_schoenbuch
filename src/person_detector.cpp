/// COMPONENT
#include "data/point.h"

#include "pillar_localization/pillar_extractor.h"

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

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace csapex;
using namespace csapex::connection_types;


namespace schoenbuch
{


namespace impl {

template <class PointT>
struct Impl;

}

class PersonDetector : public Node
{
public:
    PersonDetector()
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
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance on ring", 0.0, 0.5, 0.05, 0.001),
                            cluster_distance_ring_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance vertically", 0.0, 0.5, 0.05, 0.001),
                            cluster_distance_vertical_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance euclidean", 0.0, 0.5, 0.05, 0.001),
                            cluster_distance_euclidean_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/min cluster size", 0, 1024, 32, 1),
                            cluster_min_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster size", 0, 1024, 32, 1),
                            cluster_max_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster diameter", 0.0, 5.0, 0.2, 0.01),
                            cluster_max_diameter_);
        params.addParameter(param::ParameterFactory::declareRange("person/min points", 1, 2048, 512, 1),
                            person_min_pts_);
        params.addParameter(param::ParameterFactory::declareInterval("person/height", 0.0, 2.0, 0.5, 2.0, 0.01),
                            person_height_);
        params.addParameter(param::ParameterFactory::declareInterval("person/width", 0.0, 2.0, 0.5, 2.0, 0.01),
                            person_width_);

    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<PersonDetector>(this, msg), msg->value);
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

        std::vector<ClusteredPoint> points;
        std::vector<Cluster> clusters;
        std::vector<std::vector<Cluster*> > row_clusters;

        points.clear();
        clusters.clear();
        row_clusters.clear();

        points.resize(cloud.size());

        {
            NAMED_INTERLUDE(preparation);
            for(int col = 0; col < cols; ++col) {
                for(int row = 0; row < rows; ++row) {
                    const pcl::PointXYZI& pt = cloud.at(col, row);
                    ClusteredPoint& pt_out = points.at(row * cols + col);

                    pt_out.x = pt.x;
                    pt_out.y = pt.y;
                    pt_out.z = pt.z;
                    pt_out.intensity = pt.intensity;

                    pt_out.row = row;
                    pt_out.col = col;
                }
            }
        }


        {
            NAMED_INTERLUDE(segmentation);

            // generate segments
            clusters.reserve(points.size());
            row_clusters.resize(rows);

            for(int row = 0; row < rows; ++row) {
                ClusteredPoint* last = nullptr;
                clusters.push_back(Cluster());
                Cluster* current_cluster = &clusters.back();
                row_clusters[row].reserve(cols);
                row_clusters[row].push_back(current_cluster);

                for(int col = 0; col < cols; ++col) {
                    ClusteredPoint& current = points[row * cols + col];
                    if(std::isnan(current.x)) continue;

                    if(last) {
                        double jump_distance = std::abs(last->range() - current.range());
                        //                        double jump_distance = last->distanceXYZ(current);

                        current.jump_distance = jump_distance;

                        if(jump_distance > cluster_distance_ring_) {
                            // new cluster
                            if((int) current_cluster->pts.size() >= cluster_min_size_) {
                                clusters.push_back(Cluster());
                                current_cluster = &clusters.back();
                                row_clusters[row].push_back(current_cluster);
                            } else {
                                current_cluster->clear();
                            }
                            current_cluster->col_start = col;
                        }
                    }

                    current_cluster->add(&current);
                    current_cluster->col_end = col;

                    last = &current;
                }

                for(std::size_t i = 0, n = row_clusters[row].size(); i < n; ++i) {
                    Cluster* c  = row_clusters[row][i];
                    if((int) c->pts.size() < cluster_min_size_ || (int) c->pts.size() > cluster_max_size_) {
                        c->clear();

                    } else if(cluster_max_diameter_ > 0.0) {
                        double diameter = c->pts.front()->distanceXYZ(*c->pts.back());
                        if(diameter > cluster_max_diameter_) {
                            c->clear();
                        }
                    }
                }
            }
        }

        {
            NAMED_INTERLUDE(row_clustering);

            // cluster row based
            bool change = true;
            while(change) {
                change = false;

                for(int row = 0; row < rows; ++row) {
                    for(int row2 = row + 1; row2 < rows; ++row2) {
                        if(row == row2) continue;

                        for(std::size_t i = 0, n = row_clusters[row].size(); i < n; ++i) {
                            Cluster* c1 = row_clusters[row][i];
                            if(c1->empty()) continue;

                            for(std::size_t j = 0, n = row_clusters[row2].size(); j < n; ++j) {
                                Cluster* c2 = row_clusters[row2][j];
                                if(c1 != c2) {
                                    if(c2->empty()) continue;
                                    if(c2->col_start > c1->col_end || c1->col_start > c2->col_end) continue;

                                    if(c1->getMean().distanceXY(c2->getMean()) < cluster_distance_vertical_) {
                                        c1->merge(c2);
                                        change = true;
                                        break;
                                    }
                                }
                            }
                        }

                        for(std::vector<Cluster*>::iterator it = row_clusters[row].begin(); it != row_clusters[row].end();) {
                            if((*it)->empty()) {
                                it = row_clusters[row].erase(it);
                            } else {
                                ++it;
                            }
                        }
                    }
                }
            }
        }

        {
            NAMED_INTERLUDE(compacting);
            for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();) {
                Cluster& c = *it;
                if(c.empty()) {
                    c.clear();
                    it = clusters.erase(it);
                } else {
                    // update pointers in
                    c.reindex();
                    ++it;
                }
            }
        }

        {
            NAMED_INTERLUDE(euclidean);

            // cluster euclidean
            bool change = true;
            while(change) {
                change = false;
                for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();++it) {
                    Cluster& c1 = *it;
                    if(!c1.empty()) {
                        for(std::vector<Cluster>::iterator it2 = it + 1; it2 != clusters.end();++it2) {
                            Cluster& c2 = *it2;
                            if(!c2.empty() && c1.getMean().distanceXY(c2.getMean()) < cluster_max_diameter_ * 2) {
                                for(std::size_t k = 0, n = c1.pts.size(); k < n; ++k) {
                                    ClusteredPoint* p1 = c1.pts[k];
                                    bool merged = false;
                                    for(std::size_t l = 0, n = c2.pts.size(); l < n; ++l) {
                                        ClusteredPoint* p2 = c2.pts[l];
                                        double distance = p1->distanceXYZ(*p2);
                                        if(distance < cluster_distance_euclidean_) {
                                            c1.merge(&c2);
                                            merged = true;
                                            change = true;
                                            break;
                                        }
                                    }
                                    if(merged) break;
                                }
                            }
                        }
                    }
                }
            }
        }

        {
            NAMED_INTERLUDE(filter);
            for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();) {
                Cluster& c = *it;
                bool too_few_points = (int) c.pts.size() < person_min_pts_;
                bool wrong_height = c.getHeight() < person_height_.first || c.getHeight() > person_height_.second;
                bool wrong_width = c.getWidth() < person_width_.first || c.getWidth() > person_width_.second;
                bool remove = too_few_points || wrong_height || wrong_width;
                if(remove) {
                    c.clear();
                    it = clusters.erase(it);
                } else {
                    c.reindex();
                    ++it;
                }
            }
        }

        // normalize cluster ids
        int id = 0;
        for(std::size_t i = 0, n = clusters.size(); i < n; ++i) {
            Cluster& c = clusters[i];
            if(c.pts.size() > 0) {
                c.id = id++;
            } else {
                c.id = -1;
            }
        }


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr labeled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>& labeled_cloud = *labeled_cloud_ptr;

        labeled_cloud.header = cloud.header;
        labeled_cloud.width = cloud.width;
        labeled_cloud.height = cloud.height;
        labeled_cloud.resize(cloud.size());

        for(int col = 0; col < cols; ++col) {
            for(int row = 0; row < rows; ++row) {
                const ClusteredPoint& pt = points.at(row * cols + col);
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
    }

private:
    Input* in_;

    Output* out_;
    Output* out_marker_;

    double cluster_distance_ring_;
    double cluster_distance_vertical_;
    double cluster_distance_euclidean_;

    int cluster_min_size_;
    int cluster_max_size_;

    double cluster_max_diameter_;

    int person_min_pts_;
    std::pair<double, double> person_height_;
    std::pair<double, double> person_width_;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(PersonDetector* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        throw std::runtime_error(std::string("point type '") + type2name(typeid(PointT)) + "' not supported");
    }
};

template <>
struct Impl<pcl::PointXYZI>
{
    static void inputCloud(PersonDetector* instance, typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        instance->inputCloudImpl(cloud);
    }
};

}

}

CSAPEX_REGISTER_CLASS(schoenbuch::PersonDetector, csapex::Node)

