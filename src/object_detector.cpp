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

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}


struct Point;

struct Cluster
{
    int id = 0;
    int col_start = 0;
    int col_end = 0;

    std::vector<Point*> pts;

    void merge(Cluster* other);
    void add(Point* p);

    bool empty();

    void clear();
};

const int OBSTACLE = 1;
const int FREE = 2;
const int OBJECT = 4;
const int FILL = 16;
const int OBJECT_FILL = OBJECT + FILL;

struct Point{
    float x;
    float y;
    float z;
    float intensity;

    float jump_distance = 0.0f;

    int row;
    int col;

    int type = 0;

    Cluster* cluster = nullptr;

    double distanceXYZ(const Point& other) {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    double distanceXY(const Point& other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
    double range() {
        return std::sqrt(x*x + y*y + z*z);
    }
};

bool Cluster::empty()
{
    return pts.empty();
}

void Cluster::add(Point *p)
{
    pts.push_back(p);
    p->cluster = this;
}

void Cluster::merge(Cluster* other)
{
    for(Point* p : other->pts) {
        pts.push_back(p);
        p->cluster = this;
    }
    other->pts.clear();
}

void Cluster::clear()
{
    for(Point* p : pts) {
        p->cluster = nullptr;
    }
    pts.clear();
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

        // CLASSIFICATION
        {
            NAMED_INTERLUDE(classification);
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

                    if(row > 0 && row < rows - 1) {
                        const pcl::PointXYZI& prev = cloud.at(col, row - 1);
                        const pcl::PointXYZI& next = cloud.at(col, row + 1);

                        Eigen::Vector3d x = convert(prev);
                        Eigen::Vector3d y = convert(pt);
                        Eigen::Vector3d z = convert(next);

                        double A = 0.5 * (((z-x).cross(y-x)).norm());
                        double curv = 4 * A / ((x-y).norm() * (y-z).norm() * (z-x).norm());

                        if(A > curvature_min_area_ && curv > curvature_threshold_) {
                            pt_out.type |= OBSTACLE;

                        } else {

                            Eigen::Vector3d a = z - x;
                            Eigen::Vector3d z_proj = z;
                            z_proj(2) = x(2);
                            Eigen::Vector3d b = z_proj - x;
                            double angle = std::acos(a.dot(b) / (a.norm() * b.norm()));

                            if(std::abs(angle) > vertical_angle_threshold_) {
                                pt_out.type |= OBJECT;

                            } else {
                                pt_out.type |= FREE;
                            }

                        }
                    }
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

        //        int d_col[] { 0, 0, -1, 1, -2, 2, -3, 3, -4, 4};
        //        int d_row[] { 1, 2,  0, 0,  0, 0,  0, 0,  0, 0};

        //        std::deque<Point*> Q;
        //        for(Point& pt : points) {
        //            if(pt.type & OBJECT) {
        //                clusters.emplace_back();
        //                pt.cluster = &clusters.back();
        //                pt.cluster->id = clusters.size();
        //                pt.cluster->pts.push_back(&pt);
        //                Q.push_back(&pt);
        //            }
        //        }

        //        while(!Q.empty()) {
        //            Point* current = Q.front();
        //            Q.pop_front();

        //            for(std::size_t n = 0; n < 10; ++n) {
        //                int nrow = current->row + d_row[n];
        //                int ncol = (current->col + d_col[n] + cols) % cols;

        //                double cluster_distance = d_col[n] != 0 ? cluster_distance_ring_ : cluster_distance_vertical_;

        //                if(nrow >= 0 && nrow < rows) {
        //                    Point* neighbor = &points.at(nrow * cols + ncol);
        //                    double distance = std::abs(current->distance() - neighbor->distance());
        //                    if(distance <= cluster_distance) {
        //                        if(!(neighbor->type & OBJECT)) {
        //                            neighbor->type |= OBJECT_FILL;
        //                            neighbor->cluster = current->cluster;
        //                            neighbor->cluster->pts.push_back(neighbor);
        //                            Q.push_back(neighbor);

        //                        } else if(current->cluster != neighbor->cluster) {
        //                            current->cluster->merge(neighbor->cluster);
        //                            neighbor->cluster = current->cluster;
        //                        }
        //                    }
        //                }
        //            }
        //        }


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

                if(pt.type & OBSTACLE) {
                    //                    pt_out.r = 255;
                } else if(pt.type & OBJECT) {
                    //                    pt_out.b = 255;
                } else {
                    //                    pt_out.g = 255;
                }

                if(false) {
                    pt_out.r = std::min(1.f, pt.jump_distance / 10.f) * 255.f;
                    pt_out.g = 255 - pt_out.r;
                    pt_out.b = 0;
                } else if(pt.cluster) {
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

