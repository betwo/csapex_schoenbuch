/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/normals_message.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>

/// SYSTEM
#include <pcl/features/integral_image_normal.h>

using namespace csapex::connection_types;


namespace csapex
{

class SplitClusters : public Node
{
public:
    SplitClusters()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");
        in_indices_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Clusters");

        out_ = modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clusters");
        out_heat_ = modifier.addOutput<PointCloudMessage>("Heat Cloud");
        out_normals_ = modifier.addOutput<NormalsMessage>("Normals");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("heat_threshold", 0.0, 1.0, 0.0, 0.001), heat_threshold_);
        params.addParameter(param::ParameterFactory::declareRange("heat_scale", 0.0, 10.0, 1.0, 0.001), heat_scale_);
        params.addParameter(param::ParameterFactory::declareRange("heat_scale/normal", 0.0, 10.0, 1.0, 0.001), heat_scale_normal_);
        params.addParameter(param::ParameterFactory::declareRange("heat_scale/distance", 0.0, 10.0, 1.0, 0.001), heat_scale_distance_);

        params.addParameter(param::ParameterFactory::declareRange("heat_step", 1, 10, 1, 1), heat_step_);
        params.addParameter(param::ParameterFactory::declareBool("heat_nonmax", true), heat_non_max_);
        params.addParameter(param::ParameterFactory::declareRange("heat_nonmax/distance", 1,10,1,1), heat_non_max_distance_);


        params.addParameter(param::ParameterFactory::declareRange("normals/max_depth_change_factor",
                                                                  0.0, 0.5, 0.02, 0.001),
                            max_depth_change_factor);
        params.addParameter(param::ParameterFactory::declareRange("normals/normal_smoothing_size",
                                                                  0.0, 50.0, 10.0, 0.1),
                            normal_smoothing_size);

        params.addParameter(param::ParameterFactory::declareRange("max_index_jump", 0.0, 10.0, 3.0, 0.01),
                            max_index_jump);

        params.addParameter(param::ParameterFactory::declareRange("grow_down", 0, 32, 1, 1),
                            grow_down_);
        params.addParameter(param::ParameterFactory::declareRange("min_segment_for_merge", 0, 1024, 0, 1),
                            min_segment_for_merge);

        params.addParameter(param::ParameterFactory::declareRange("min_overlap", 0, 32, 0, 1),
                            min_overlap);
        params.addParameter(param::ParameterFactory::declareRange("merge_distance_threshold", 0.0, 2.0, 0.1, 0.001),
                            merge_distance_threshold_);
        params.addParameter(param::ParameterFactory::declareRange("min_segment_size", 0, 1024, 0, 1),
                            segment_min_size_);
        params.addParameter(param::ParameterFactory::declareRange("max_segment_size", 0, 1024, 1024, 1),
                            segment_max_size_);
        params.addParameter(param::ParameterFactory::declareRange("min_cluster_size", 0, 1024, 0, 1),
                            cluster_min_size_);
        params.addParameter(param::ParameterFactory::declareRange("max_cluster_size", 0, 1024 * 10, 1024 * 4, 1),
                            cluster_max_size_);


        typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> T;

        std::map<std::string, int> methods {
            {"COVARIANCE_MATRIX", T::COVARIANCE_MATRIX},
            {"AVERAGE_3D_GRADIENT", T::AVERAGE_3D_GRADIENT},
            {"AVERAGE_DEPTH_CHANGE", T::AVERAGE_DEPTH_CHANGE},
            {"SIMPLE_3D_GRADIENT", T::SIMPLE_3D_GRADIENT}
        };
        params.addParameter(param::ParameterFactory::declareParameterSet("normals/method", methods,
                                                                         (int) T::AVERAGE_3D_GRADIENT),
                            method);
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<SplitClusters>(this, msg), msg->value);

        PointCloudMessage::Ptr heat_cloud_msg = std::make_shared<PointCloudMessage>(msg->frame_id, msg->stamp_micro_seconds);
        heat_cloud_msg->value = heat_cloud;
        msg::publish(out_heat_, heat_cloud_msg);

        NormalsMessage::Ptr normals_msg = std::make_shared<NormalsMessage>(msg->frame_id, msg->stamp_micro_seconds);
        normals_msg->value = normals;
        msg::publish(out_normals_, normals_msg);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;

        normals.reset(new pcl::PointCloud<pcl::Normal>);

        {
            INTERLUDE("normal estimation");

            pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
            ne.setNormalEstimationMethod (static_cast<typename pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::NormalEstimationMethod>(method));
            ne.setMaxDepthChangeFactor(max_depth_change_factor);
            ne.setNormalSmoothingSize(normal_smoothing_size);
            ne.setInputCloud(cloud_ptr);
            ne.compute(*normals);
        }

        heat_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(cloud.width, cloud.height);
        heat_cloud->header = cloud.header;
        {
            const PointT* pt_from = &cloud.points.front();
            pcl::PointXYZRGB* pt_to = &heat_cloud->points.front();
            for(std::size_t i = 0, n = cloud.points.size(); i < n; ++i, ++pt_from, ++pt_to) {
                pt_to->x = pt_from->x;
                pt_to->y = pt_from->y;
                pt_to->z = pt_from->z;
            }
        }

        auto cluster_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);
        const std::vector<pcl::PointIndices>& indices = *cluster_indices;

        auto out_indices_msg = std::make_shared<std::vector<pcl::PointIndices>>();
        std::vector<pcl::PointIndices>& split_indices = *out_indices_msg;



        heat = std::vector<double>(cloud.points.size(), 0.0);

        for(const pcl::PointIndices& cluster : indices) {
            std::vector<pcl::PointIndices> split = splitCluster(cloud, cluster);
            split_indices.insert(split_indices.end(), split.begin(), split.end());
        }

        {
            double* heat_p = &heat.front();
            pcl::PointXYZRGB* pt_to = &heat_cloud->points.front();
            for(std::size_t i = 0, n = cloud.points.size(); i < n; ++i, ++heat_p, ++pt_to) {
                // hue should range from 120° (green) to 0° (red)
                // mapping: heat = 0 -> 120°, heat >> 0 -> 0°

                double H = std::min(1.0, std::max(0.0, *heat_p));

                double h = 120 - (std::min(H * 120.0, 120.0));
                double s = (H > heat_threshold_) ? 255 : 128;
                double v = (H > heat_threshold_) ? 255 : 128;

                double r = 0, g = 0, b = 0;
                __HSV2RGB__(h,s,v, r,g,b);
                pt_to->r = r * 255;
                pt_to->g = g * 255;
                pt_to->b = b * 255;
            }
        }
        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_, out_indices_msg);
    }


    template <class PointT>
    std::vector<pcl::PointIndices> splitCluster(const pcl::PointCloud<PointT>& cloud, const pcl::PointIndices& cluster)
    {
        INTERLUDE("main");

        std::vector<pcl::PointIndices> res;

        {
            INTERLUDE("heat");
            calculateHeatClusters(cloud, cluster);
        }

        if(heat_non_max_) {
            INTERLUDE("non maximum suppression");
            nonMaxSuppression(cloud.width, cluster);
        }

        INTERLUDE("split");
        split(cloud, cluster, res);

        return res;
    }

    template <class PointT>
    void split(const pcl::PointCloud<PointT>& cloud, const pcl::PointIndices& cluster, std::vector<pcl::PointIndices>& res)
    {
        //        res.push_back(cluster);

        // make sure that the indices are sorted -> we want to iterate row by row
        std::vector<int> indices = cluster.indices;
        std::sort(indices.begin(), indices.end());

        class Segment;

        struct Cluster {
            Cluster(){}

            bool empty() const
            {
                return segments.empty();
            }

            void remove(Segment* s) {
                segments.erase(std::find(segments.begin(), segments.end(), s));
            }

            void merge(Cluster* other)
            {
                apex_assert(other != this);
                for(Segment* s : other->segments) {
                    segments.push_back(s);
                    s->cluster = this;
                }
                other->segments.clear();
            }

            std::vector<Segment*> segments;
        };

        struct Segment {
            const int row;

            int start;
            int end;

            Segment(int row, int start)
                : row(row), start(start), end(start), cluster(nullptr)
            {

            }

            bool empty() const
            {
                return end <= start;
            }

            void merge(Segment* other) {
                apex_assert(other != this);
                cluster->merge(other->cluster);
                other->cluster = cluster;
            }

            void setCluster(Cluster* c) {
                cluster = c;
                c->segments.push_back(this);
            }

            Cluster* cluster;
        };

        std::vector<Cluster> sub_clusters;
        sub_clusters.reserve(cluster.indices.size());
        std::map<int, std::vector<Segment>> segments;

        // generate segments
        Segment* current_row_cluster = nullptr;

        int cols = cloud.width;
        int first_row = indices.front() / cols;
        int last_row = -1;
        int last_col = -1;
        for(int index : indices) {
            int col = index % cols;
            int row = index / cols;

            if(row != last_row) {
                if(current_row_cluster) {
                    int ssize = current_row_cluster->end - current_row_cluster->start;
                    if(ssize < segment_min_size_ || ssize > segment_max_size_) {
                        current_row_cluster->cluster->remove(current_row_cluster);
                        segments[last_row].pop_back();
                    }
                }
                segments[row].reserve(cluster.indices.size());
                current_row_cluster = nullptr;
            }

            double index_jump = std::hypot(row - last_row, col - last_col);

            last_row = row;
            last_col = col;

            double H = heat.at(index);
            bool jumped = index_jump > max_index_jump;
            bool in_segment = H < heat_threshold_;

            if(current_row_cluster && !jumped) {
                current_row_cluster->end = col;
            }

            if(!in_segment || jumped) {
                if(current_row_cluster) {
                    int ssize = current_row_cluster->end - current_row_cluster->start;
                    if(ssize < segment_min_size_ || ssize > segment_max_size_) {
                        current_row_cluster->cluster->remove(current_row_cluster);
                        segments[row].pop_back();
                    }
                    current_row_cluster = nullptr;
                }
            }

            if(in_segment) {
                if(!current_row_cluster) {
                    // new row cluster
                    segments[row].emplace_back(row, col);
                    current_row_cluster = &segments[row].back();

                    sub_clusters.emplace_back();

                    current_row_cluster->setCluster(&sub_clusters.back());
                }

                current_row_cluster->end = col;

            }
        }

        if(current_row_cluster) {
            int ssize = current_row_cluster->end - current_row_cluster->start;
            if(ssize < segment_min_size_ || ssize > segment_max_size_) {
                current_row_cluster->cluster->remove(current_row_cluster);
                segments[last_row].pop_back();
            }
        }


        {
            double merge_distance_threshold_sqr = merge_distance_threshold_ * merge_distance_threshold_;

            // cluster row based
            bool change = true;
            while(change) {
                change = false;
                int d = grow_down_;
                for(int row = first_row + d; row < last_row - d; ++row) {
//                    for(int row2 = row + 1; row2 < last_row; ++row2) {
                    for(int row2 = row + 1; row2 <= row + d; ++row2) {
//                        for(int row2 = row - d; row2 <= row + d; ++row2) {
                        if(row == row2) continue;

                        for(std::size_t i = 0, n = segments[row].size(); i < n; ++i) {
                            Segment* c1 = &segments[row][i];
                            if(c1->empty()) continue;

                            int l1 = c1->end - c1->start;
                            if(l1 < min_segment_for_merge) {
                                continue;
                            }

                            for(std::size_t j = 0, n = segments[row2].size(); j < n; ++j) {
                                Segment* c2 = &segments[row2][j];
                                if(c1 != c2 && c1->cluster != c2->cluster) {
                                    if(c2->empty()) continue;
                                    if(c2->start > c1->end || c1->start > c2->end) continue;

                                    int l2 = c2->end - c2->start;

//                                    int overlap = (c1->start >= c2->start) ? (c2->end - c1->start) : (c1->end - c2->start);
                                    int overlap = std::min(c1->end, c2->end) - std::max(c1->start, c2->start) + 1;

                                    if(overlap >= std::min(min_overlap, std::min(l1, l2))) {
//                                        c1->merge(c2);
//                                        change = true;
//                                        //break;

                                        int r1 = c1->row * cloud.width;
                                        int r2 = c2->row * cloud.width;

                                        int offset = 1;

                                        for(int k = c1->start + offset, n = c1->end - offset; k < n; ++k) {
                                            const PointT& p1 = cloud.points.at(r1 + k);
                                            bool merged = false;
                                            for(int l = c2->start + offset, m = c2->end - offset; l < m; ++l) {
                                                const PointT& p2 = cloud.points.at(r2 + l);
                                                double distance_sqr = std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2);
                                                if(distance_sqr  < merge_distance_threshold_sqr) {
                                                    c1->merge(c2);
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
            }
        }

        // generate clusters
        for(Cluster& c : sub_clusters) {
            if(c.empty()) {
                continue;
            }

            pcl::PointIndices cluster;
            for(Segment* s : c.segments) {
                for(int col = s->start, e = s->end; col <= e; ++col) {
                    cluster.indices.push_back(s->row * cols + col);
                }
            }
            if(cluster.indices.size() >= cluster_min_size_ &&
                    cluster.indices.size() <= cluster_max_size_) {
                res.push_back(cluster);
            }
        }
    }

    template <class PointT>
    void calculateHeatClusters(const pcl::PointCloud<PointT>& cloud, const pcl::PointIndices& cluster)
    {

        static const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

        int w = cloud.width;
        int h = cloud.height;

        for(int index : cluster.indices) {
            const pcl::Normal& normal = normals->at(index);
            bool has_normal = pcl::isFinite(normal);

            Eigen::Vector3d normal_v(normal.normal_x, normal.normal_y, normal.normal_z);

            const PointT pt = cloud.at(index);
            Eigen::Vector3d pt_v(pt.x, pt.y, pt.z);

            int x = index % w;
            int y = index / w;

            apex_assert(y * w + x == index);

            double normal_heat = -1.0;
            double distance_heat = 0.0;

            for(std::size_t i = 0, d = 8; i < d; ++i) {
                int nx = x + dx[i] * heat_step_;
                int ny = y + dy[i] * heat_step_;

                if(nx >= 0 && nx < w && ny >= 0 && ny < h) {
                    int n_index = ny * w + nx;
                    const pcl::Normal& n_normal = normals->at(n_index);

                    if(has_normal && pcl::isFinite(n_normal)) {
                        Eigen::Vector3d n_normal_v(n_normal.normal_x, n_normal.normal_y, n_normal.normal_z);
                        double angle = std::acos(normal_v.dot(n_normal_v));

                        normal_heat = std::max(normal_heat, std::abs(angle) / M_PI);
                    }

                    const PointT n_pt = cloud.at(n_index);
                    if(pcl::isFinite(n_pt)) {
                        Eigen::Vector3d n_pt_v(n_pt.x, n_pt.y, n_pt.z);
                        double distance = (pt_v - n_pt_v).norm();
                        distance_heat = std::max(distance_heat, std::abs(distance));
                    }
                }
            }

            if(normal_heat < 0.0) {
                // no valid neighbor
                if(has_normal) {
                    normal_heat = 1.0;
                } else {
                    normal_heat = 0.5;
                }
            }

            double H = normal_heat * heat_scale_normal_ +
                    distance_heat * heat_scale_distance_;

            heat.at(index) = H * heat_scale_;
        }
    }


    void nonMaxSuppression(int cloud_width, const pcl::PointIndices& cluster)
    {
        for(int index : cluster.indices) {
            int x = index % cloud_width;
            int y = index / cloud_width;

            double max = 0.0;

            for(int dx = -heat_non_max_distance_; dx < heat_non_max_distance_; ++dx) {
                int nx = x + dx;

                if(nx >= 0 && nx < cloud_width) {
                    int n_index = y * cloud_width + nx;
                    max = std::max(max, heat.at(n_index));
                }
            }

            double& H = heat.at(index);
            if(max > H) {
                H = 0.0;
            }
        }
    }

private:
    Input* in_;
    Input* in_indices_;

    Output* out_;
    Output* out_heat_;
    Output* out_normals_;

    std::vector<double> heat;

    double heat_threshold_;
    double heat_scale_;
    double heat_scale_normal_;
    double heat_scale_distance_;

    bool heat_non_max_;
    int heat_non_max_distance_;
    int heat_step_;

    int method;
    double max_depth_change_factor;
    double normal_smoothing_size;

    double max_index_jump;
    int grow_down_;
    int min_segment_for_merge;
    int min_overlap;
    double merge_distance_threshold_;

    int segment_min_size_;
    int segment_max_size_;
    int cluster_min_size_;
    int cluster_max_size_;

    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heat_cloud;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(SplitClusters* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        instance->inputCloud(cloud);
    }
};

}

}

CSAPEX_REGISTER_CLASS(csapex::SplitClusters, csapex::Node)

