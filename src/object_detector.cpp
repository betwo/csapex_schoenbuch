/// COMPONENT
#include "data/point.h"
#include "data/cluster.h"

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
        params.addParameter(param::ParameterFactory::declareRange("curvature/threshold", 0.0, 4.0, 1.0, 0.01),
                            curvature_threshold_);
        params.addParameter(param::ParameterFactory::declareRange("curvature/min area", 0.0, 1.0, 0.0, 0.01),
                            curvature_min_area_);


        params.addParameter(param::ParameterFactory::declareAngle("vertical/angle threshold", M_PI_4),
                            vertical_angle_threshold_);


        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance on ring", 0.0, 0.1, 0.05, 0.001),
                            cluster_distance_ring_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max distance vertically", 0.0, 0.1, 0.05, 0.001),
                            cluster_distance_vertical_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/min cluster size", 0, 1024, 32, 1),
                            cluster_min_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster size", 0, 1024, 32, 1),
                            cluster_max_size_);
        params.addParameter(param::ParameterFactory::declareRange("clustering/max cluster diameter", 0.0, 5.0, 0.2, 0.01),
                            cluster_max_diameter_);

        params.addParameter(param::ParameterFactory::declareRange("pillar/min intensity", 0.0, 1024.0, 120.0, 0.1),
                            pillar_min_intensity_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/min points", 0, 512, 32, 1),
                            pillar_min_points_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/radius", 0.001, 1.0, 0.055, 0.001),
                            pillar_radius_);
        params.addParameter(param::ParameterFactory::declareRange("pillar/radius threshold", 0.000, 1.0, 0.055, 0.001),
                            pillar_radius_fuzzy_);

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


        std::vector<Point> points;
        points.resize(cloud.size());

        {
            NAMED_INTERLUDE(convert);
            for(int col = 0; col < cols; ++col) {
                for(int row = 0; row < rows; ++row) {
                    const pcl::PointXYZI& pt = cloud.at(col, row);
                    Point& pt_out = points.at(row * cols + col);

                    pt_out.x = pt.x;
                    pt_out.y = pt.y;
                    pt_out.z = pt.z;
                    pt_out.intensity = pt.intensity;

                    pt_out.row = row;
                    pt_out.col = col;
                }
            }
        }

        std::vector<Cluster> clusters;
        std::vector<std::vector<Cluster*>> row_clusters;

        {
            // CLUSTERING: seed with points marked as "vertical"
            NAMED_INTERLUDE(cluster);

            clusters.reserve(points.size());
            row_clusters.resize(rows);

            for(int row = 0; row < rows; ++row) {
                Point* last = nullptr;
                clusters.emplace_back();
                Cluster* current_cluster = &clusters.back();
                row_clusters[row].reserve(cols);
                row_clusters[row].push_back(current_cluster);

                for(int col = 0; col < cols; ++col) {
                    Point& current = points[row * cols + col];
                    if(std::isnan(current.x)) continue;

                    if(last) {
                        double jump_distance = std::abs(last->range() - current.range());
                        //                        double jump_distance = last->distanceXYZ(current);

                        current.jump_distance = jump_distance;

                        if(jump_distance > cluster_distance_ring_) {
                            // new cluster
                            if(current_cluster->pts.size() >= cluster_min_size_) {
                                clusters.emplace_back();
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

                for(Cluster* c : row_clusters[row]) {
                    if(c->pts.size() < cluster_min_size_ || c->pts.size() > cluster_max_size_) {
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
            NAMED_INTERLUDE(merge_cluster);

            bool change = true;
            while(change) {
                change = false;

                for(int row = 0; row < rows; ++row) {
                    for(int row2 = row + 1; row2 < rows; ++row2) {
                        if(row == row2) continue;

                        for(Cluster* c1 : row_clusters[row]) {
                            if(c1->empty()) continue;

                            for(Cluster* c2 : row_clusters[row2]) {
                                if(c1 != c2) {
                                    if(c2->empty()) continue;
                                    if(c2->col_start > c1->col_end || c1->col_start > c2->col_end) continue;

                                    for(Point* p1 : c1->pts) {
                                        bool merged = false;
                                        for(Point* p2 : c2->pts) {
                                            double distance = p1->distanceXY(*p2);
                                            if(distance < cluster_distance_vertical_) {
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

                        for(auto it = row_clusters[row].begin(); it != row_clusters[row].end();) {
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

        // filter intensity
        for(Cluster& c : clusters) {
            if(c.pts.size() < pillar_min_points_) {
                c.clear();
            } else {
                std::vector<float> intensities(c.pts.size(), 0.0f);
                std::size_t i = 0;
                for(Point* p : c.pts) {
                    intensities[i++] = p->intensity;
                }

                std::sort(intensities.begin(), intensities.end());

                if(intensities.back() < pillar_min_intensity_) {
                    c.clear();
                }
            }
        }


        // normalize cluster ids
        int id = 0;
        for(Cluster& c : clusters) {
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
                const Point& pt = points.at(row * cols + col);
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

            for(Cluster& c : clusters) {
                if(c.empty()) {
                    continue;
                }

                double r;
                Eigen::Vector3d C, W;
                double error = fitCylinder(c.pts, r, C, W);

                if(error < 1e-5 &&
                        r > pillar_radius_ - pillar_radius_fuzzy_ &&
                        r < pillar_radius_ + pillar_radius_fuzzy_) {

                    marker.pose.position.x = C(0);
                    marker.pose.position.y = C(1);
                    marker.pose.position.z = C(2);

                    tf::Matrix3x3 base_T;
                    tf::Vector3 z(W(0), W(1), W(2));
                    tf::Vector3 x(1, 0, 0);
                    tf::Vector3 y = z.cross(x);
                    base_T[0] = x;
                    base_T[1] = y;
                    base_T[2] = z;

                    //tf::Quaternion base(W(0),W(1),W(2),0);
                    tf::Quaternion base;
                    base_T.getRotation(base);
                    tf::quaternionTFToMsg(base, marker.pose.orientation);

                    marker.scale.x = r * 2;
                    marker.scale.y = r * 2;
                    marker.scale.z = 2.0;

                    marker.id = id++;
                    marker_array.markers.push_back(marker);
                }
            }

            msg::publish(out_marker_, message);
        }
    }

    double fitCylinder(const std::vector<Point*>& cluster,
                       double& r, Eigen::Vector3d& C, Eigen::Vector3d& W)
    {
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> X;
        std::size_t n = cluster.size();

        X.reserve(n);

        Eigen::Vector3d average = Eigen::Vector3d::Zero();
        for(Point* pt : cluster) {
            Eigen::Vector3d p;
            p << pt->x, pt->y, pt->z;

            X.push_back(p);

            average += p;
        }
        average /= (double) n;
        for(Eigen::Vector3d& pt : X) {
            pt -= average;
        }

        double min_error = std::numeric_limits<double>::infinity();
        double rsqr = 0.0;

        int i_max = 64;
        int j_max = 64;

        for(int j = 0; j < j_max; ++j) {
            double phi = M_PI_2 * j / (double) j_max;
            double cos_phi = std::cos(phi);
            double sin_phi = std::sin(phi);

            for(int i = 0; i < i_max; ++i) {
                double theta = 2 * M_PI * i / (double) i_max;
                double cos_theta = std::cos(theta);
                double sin_theta = std::sin(theta);

                Eigen::Vector3d currentW(cos_theta * sin_phi, sin_theta * sin_phi, cos_phi);

                Eigen::Vector3d currentC;
                double current_rsqr;

                double error = G(n, X, currentW, currentC, current_rsqr);

                if(error < min_error) {
                    min_error = error;
                    W = currentW;
                    C = currentC;
                    rsqr = current_rsqr;
                }
            }
        }

        C += average;

        r = std::sqrt(rsqr);

        return min_error;
    }

    double G(std::size_t n,
             const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& X,
             const Eigen::Vector3d& W,
             Eigen::Vector3d& PC,
             double& rsqr)
    {
        Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - W * W.transpose();
        Eigen::Matrix3d S;
        S << 0, -W(2), W(1),
                0, -W(0), -W(1),
                -W(1), W(0), 0;
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d B = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Y(n);
        double averageSqrLength = 0;
        std::vector<double> sqrLength(n);

        for(std::size_t i = 0; i < n; ++i) {
            Y[i] = P * X[i];
            sqrLength[i] = Y[i].dot(Y[i]);

            A += Y[i] * Y[i].transpose();
            B += sqrLength[i] * Y[i];

            averageSqrLength += sqrLength[i];
        }

        A /= (double) n;
        B /= (double) n;

        averageSqrLength /= n;

        Eigen::Matrix3d  Ahat = -S * A * S;
        PC = (Ahat * B) / (Ahat * A).trace();

        double error = 0.0;
        rsqr = 0.0;

        for(std::size_t i = 0; i < n; ++i) {
            double term = sqrLength[i] - averageSqrLength - 2 * Y[i].dot(PC);
            error += term * term;
            Eigen::Vector3d diff = PC - Y[i];
            rsqr += diff.dot(diff);
        }

        error /= n;
        rsqr /= n;

        return error;
    }

private:
    Input* in_;

    Output* out_;
    Output* out_marker_;


    double curvature_threshold_;
    double curvature_min_area_;

    double vertical_angle_threshold_;

    double cluster_distance_ring_;
    double cluster_distance_vertical_;

    int cluster_min_size_;
    int cluster_max_size_;

    double cluster_max_diameter_;

    double pillar_min_intensity_;
    int pillar_min_points_;

    double pillar_radius_;
    double pillar_radius_fuzzy_;
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

