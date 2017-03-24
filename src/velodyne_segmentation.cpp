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
#include <pcl/filters/voxel_grid.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}

class VelodyneSegmentation : public Node
{
public:
    VelodyneSegmentation()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");

        out_ = modifier.addOutput<PointCloudMessage>("Segment");
        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("Markers");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("max floor height",
                                                                  -5.0, 5.0, -0.25, 0.01),
                            max_floor_height_);
        params.addParameter(param::ParameterFactory::declareAngle("max floor angle",
                                                                   M_PI / 4),
                            max_floor_angle_);
        params.addParameter(param::ParameterFactory::declareAngle("min obstacle angle",
                                                                   M_PI / 3),
                            min_obstacle_angle_);

        params.addParameter(param::ParameterFactory::declareRange("skip", 1, 32, 1, 1), skip_);

        params.addParameter(param::ParameterFactory::declareBool("keep_organized", false), keep_organized_);
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        typename pcl::PointCloud<pcl::PointXYZI>::Ptr input = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(msg->value);
        if(input) {
            const pcl::PointCloud<pcl::PointXYZI>& cloud = *input;
            pcl::PointCloud<pcl::PointXYZI>::Ptr res(new pcl::PointCloud<pcl::PointXYZI>);
            res->header = input->header;

            convertToMesh(cloud, *res);
            //segment(cloud, *res);

            if(keep_organized_) {
                res->width = res->points.size() / 16;
                res->height = 16;
                apex_assert(res->width * res->height == res->points.size());
                res->is_dense = true;
            } else {
                res->width = res->points.size();
                res->height = 1;
                res->is_dense = false;
            }

            PointCloudMessage::Ptr output_msg(new PointCloudMessage(cloud.header.frame_id, cloud.header.stamp));
            output_msg->value = res;

            msg::publish(out_, output_msg);
        }
    }

    void convertToMesh(const pcl::PointCloud<pcl::PointXYZI>& in, pcl::PointCloud<pcl::PointXYZI>& out)
    {
        apex_assert(in.isOrganized());

        visualization_msgs::Marker floor;
        floor.header.frame_id = in.header.frame_id;
        floor.header.stamp.fromNSec(in.header.stamp * 1e3);
        floor.action = visualization_msgs::Marker::ADD;
        floor.type = visualization_msgs::Marker::TRIANGLE_LIST;
        floor.ns = "floor";
        floor.pose.orientation.w = 1.0;
        floor.color.r = 1.0;
        floor.color.a = 0.7;
        floor.pose.orientation.w = 1.0;
        floor.scale.x = 1.0;
        floor.scale.y = 1.0;
        floor.scale.z = 1.0;

        visualization_msgs::Marker objects;
        objects = floor;
        objects.ns = "objects";

        pcl::PointXYZI nan;
        nan.x = nan.y = nan.z = nan.intensity = std::numeric_limits<float>::quiet_NaN();

        auto add_triangle = [this, &floor, &objects, &out, nan](const pcl::PointXYZI& ca, const pcl::PointXYZI& cb, const pcl::PointXYZI& cc) -> boost::optional<pcl::PointXYZI>
        {
            tf::Vector3 a(ca.x, ca.y, ca.z);
            tf::Vector3 b(cb.x, cb.y, cb.z);
            tf::Vector3 c(cc.x, cc.y, cc.z);

            tf::Vector3 z(0,0,1);

            tf::Vector3 n = ((b - a).cross(c - a)).normalized();

            double tilt = std::abs(std::atan2(n.cross(z).length(),n.dot(z)));
            if(tilt > M_PI_2) {
                tilt = M_PI - tilt;
            }


            geometry_msgs::Point pa;
            pa.x = ca.x; pa.y = ca.y; pa.z = ca.z;
            geometry_msgs::Point pb;
            pb.x = cb.x; pb.y = cb.y; pb.z = cb.z;
            geometry_msgs::Point pc;
            pc.x = cc.x; pc.y = cc.y; pc.z = cc.z;


            double mz = std::min(pa.z, std::min(pb.z, pc.z));

//            bool inserted = false;

            if(tilt < max_floor_angle_) {

                if(mz < max_floor_height_) {
                    floor.points.push_back(pa);
                    floor.points.push_back(pb);
                    floor.points.push_back(pc);

                    std_msgs::ColorRGBA color;
                    color.r = 0.0;
                    color.g = 0.7;
                    color.b = 0.0;
                    color.a = 1.0;

                    floor.colors.push_back(color);
                    floor.colors.push_back(color);
                    floor.colors.push_back(color);
                }

            } else if(tilt > min_obstacle_angle_) {
                //if(mz > -0.25) {
                    objects.points.push_back(pa);
                    objects.points.push_back(pb);
                    objects.points.push_back(pc);

                    std_msgs::ColorRGBA color;
                    color.r = 0.7;
                    color.g = 0.0;
                    color.b = 0.0;
                    color.a = 1.0;

                    objects.colors.push_back(color);
                    objects.colors.push_back(color);
                    objects.colors.push_back(color);

                    double da = a.length();
                    double db = b.length();
                    double dc = c.length();

                    double m = std::min(da, std::min(db, dc));

                    pcl::PointXYZI o;
                    if(m == da) {
                        o = ca;
                    } else if(m == db) {
                        o = cb;
                    } else {
                        o = cc;
                    }

//                    out.points.push_back(o);
                    return o;
               // }
            }

//            if(keep_organized_ && !inserted) {
//                out.points.push_back(nan);
//            }

            return boost::optional<pcl::PointXYZI>();
        };

        for(int row = 0, rows = in.height; row < rows; ++row) {
            for(int col = 0, cols = in.width; col <= cols-skip_; col += skip_) {
                if(row == rows - 1) {
                    if(keep_organized_) {
                        out.points.push_back(nan);
                    }
                    continue;
                }

                const pcl::PointXYZI& bl = in.at(col, row);
                const pcl::PointXYZI& br = in.at((col+skip_)%cols, row);
                const pcl::PointXYZI& tl = in.at(col, row+1);
                const pcl::PointXYZI& tr = in.at((col+skip_)%cols, row+1);

                boost::optional<pcl::PointXYZI> inserted;
                if(pcl::isFinite(bl) && pcl::isFinite(tl) && pcl::isFinite(tr)) {
                    inserted = add_triangle(bl, tl, tr);
                }
                if(!inserted) {
                    if(pcl::isFinite(bl) && pcl::isFinite(tr) && pcl::isFinite(br)) {
                        inserted = add_triangle(bl, tr, br);
                    }
                }

                if(inserted) {
                    //out.points.push_back(inserted.get());
                    out.points.push_back(bl);
                } else if(keep_organized_) {
                    out.points.push_back(nan);
                }
            }
        }



        std::shared_ptr<visualization_msgs::MarkerArray> markers = std::make_shared<visualization_msgs::MarkerArray>();
        markers->markers.push_back(floor);
        markers->markers.push_back(objects);
        msg::publish(out_marker_, markers);
    }

    void segment(const pcl::PointCloud<pcl::PointXYZI>& in, pcl::PointCloud<pcl::PointXYZI>& out)
    {
        double dimensions = 50.0 /*m*/;
        double resolution = 2.5;
        int size = std::ceil(dimensions / resolution);

        int rows = size;
        int cols = size;

        int half_rows = rows / 2;
        int half_cols = cols / 2;

        struct Bin {
            std::vector<pcl::PointXYZI> pts;
            pcl::PointXYZI* representative = nullptr;

            tf::Vector3 normal;
        };

        std::vector<Bin> bins(size * size);


        for(std::size_t idx = 0, n = in.size(); idx < n; ++idx) {
            const pcl::PointXYZI& pt = in.points.at(idx);
            int col = pt.x / resolution + half_cols;
            int row = pt.y / resolution + half_rows;
            if(col >= 0 && col < cols && row >= 0 && row < cols) {
                Bin& bin = bins.at(row * cols + col);
                bin.pts.push_back(pt);
            }
        }

        Bin* next_bin = &bins.front();
        for(int row = 0; row < rows; ++row) {
            for(int col = 0; col < cols; ++col, ++next_bin) {
                Bin& bin = *next_bin;
                if(bin.pts.size() > 3) {
                    std::sort(bin.pts.begin(), bin.pts.end(),
                              [](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
                        return a.z < b.z;
                    });

                    bin.representative = &bin.pts.front();

                    tf::Vector3 a(bin.pts[0].x, bin.pts[0].y, bin.pts[0].z);
                    tf::Vector3 b(bin.pts[1].x, bin.pts[1].y, bin.pts[1].z);
                    tf::Vector3 c(bin.pts[2].x, bin.pts[2].y, bin.pts[2].z);

                    bin.normal = ((b - a).cross(c - a)).normalized();
                }
            }
        }



        visualization_msgs::Marker floor;
        floor.header.frame_id = in.header.frame_id;
        floor.header.stamp.fromNSec(in.header.stamp * 1e3);
        floor.action = visualization_msgs::Marker::ADD;
        floor.type = visualization_msgs::Marker::TRIANGLE_LIST;
        floor.ns = "floor";
        floor.pose.orientation.w = 1.0;
        floor.color.r = 1.0;
        floor.color.a = 0.7;
        floor.pose.orientation.w = 1.0;
        floor.scale.x = 1.0;
        floor.scale.y = 1.0;
        floor.scale.z = 1.0;

//        auto calc_z = [](const Eigen::Hyperplane<double, 3>& plane, geometry_msgs::Point& pt){
//            // ax + by + cz + d = 0
//            // -> z = (-ax - by - d) / c
//            auto coeffs = plane.coeffs();
//            pt.z = (-coeffs[0] * pt.x - coeffs[1] * pt.y - coeffs[3]) / coeffs[2];
//            //            Eigen::Vector3d ptl_proj = plane.projection(Eigen::Vector3d(pt.x,pt.y,0));
//            //            pt.z = ptl_proj(2);
//        };

//        Eigen::Vector3d z = Eigen::Vector3d::UnitZ();

        //        for(int row = 1; row < rows; ++row) {
        //            for(int col = 1; col < cols; ++col) {
        //                Bin& bin = bins.at((row) * cols + (col));
        //                if(bin.representative) {

        //                    Eigen::Vector3d normal(bin.normal.x(), bin.normal.y(), bin.normal.z());
        //                    Eigen::Vector3d e(bin.representative->x, bin.representative->y, bin.representative->z);
        //                    Eigen::Hyperplane<double, 3> plane(normal, e);


        //                    geometry_msgs::Point ptl;
        //                    ptl.x = (col - half_cols) * resolution;
        //                    ptl.y = (row - half_rows) * resolution;
        //                    calc_z(plane, ptl);

        //                    geometry_msgs::Point ptr;
        //                    ptr.x = (col+1 - half_cols) * resolution;
        //                    ptr.y = (row - half_rows) * resolution;
        //                    calc_z(plane, ptr);

        //                    geometry_msgs::Point pbl;
        //                    pbl.x = (col - half_cols) * resolution;
        //                    pbl.y = (row+1 - half_rows) * resolution;
        //                    calc_z(plane, pbl);

        //                    geometry_msgs::Point pbr;
        //                    pbr.x = (col+1 - half_cols) * resolution;
        //                    pbr.y = (row+1 - half_rows) * resolution;
        //                    calc_z(plane, pbr);

        //                    floor.points.push_back(ptl);
        //                    floor.points.push_back(ptr);
        //                    floor.points.push_back(pbl);

        //                    floor.points.push_back(pbl);
        //                    floor.points.push_back(ptr);
        //                    floor.points.push_back(pbr);

        //                    double tilt = std::atan2(normal.cross(z).norm(),normal.dot(z));
        //                    if(tilt > M_PI_2) {
        //                        tilt = M_PI - tilt;
        //                    }

        //                    std_msgs::ColorRGBA c;
        //                    c.r = tilt / M_PI_2;
        //                    c.g = 1.0 - c.r;
        //                    c.b = 0.0;
        //                    c.a = 1.0;

        //                    floor.colors.push_back(c);
        //                    floor.colors.push_back(c);
        //                }
        //            }
        //        }

        auto add_triangle = [&floor](const geometry_msgs::Point& pa, const geometry_msgs::Point& pb, const geometry_msgs::Point& pc) {

            floor.points.push_back(pa);
            floor.points.push_back(pb);
            floor.points.push_back(pc);

            tf::Vector3 a(pa.x, pa.y, pa.z);
            tf::Vector3 b(pb.x, pb.y, pb.z);
            tf::Vector3 c(pc.x, pc.y, pc.z);

            tf::Vector3 z(0,0,1);

            tf::Vector3 n = (b - a).cross(c - a).normalized();

            double tilt = std::atan2(n.cross(z).length(),n.dot(z));
            if(tilt > M_PI_2) {
                tilt = M_PI - tilt;
            }

            std_msgs::ColorRGBA color;
            color.r = tilt / M_PI_2;
            color.g = 1.0 - color.r;
            color.b = 0.0;
            color.a = 1.0;

            floor.colors.push_back(color);
        };

        for(int row = 1; row < rows; ++row) {
            for(int col = 1; col < cols; ++col) {
                Bin& tl = bins.at((row-1) * cols + (col-1));
                Bin& tr = bins.at((row-1) * cols + (col));
                Bin& bl = bins.at((row) * cols + (col-1));
                Bin& br = bins.at((row) * cols + (col));

                if(tl.representative && tr.representative && bl.representative && br.representative) {

                    geometry_msgs::Point ptl;
                    ptl.x = (col-1 - half_cols) * resolution;
                    ptl.y = (row-1 - half_rows) * resolution;
                    ptl.z = tl.representative->z;

                    geometry_msgs::Point ptr;
                    ptr.x = (col - half_cols) * resolution;
                    ptr.y = (row-1 - half_rows) * resolution;
                    ptr.z = tr.representative->z;

                    geometry_msgs::Point pbl;
                    pbl.x = (col-1 - half_cols) * resolution;
                    pbl.y = (row - half_rows) * resolution;
                    pbl.z = bl.representative->z;

                    geometry_msgs::Point pbr;
                    pbr.x = (col - half_cols) * resolution;
                    pbr.y = (row - half_rows) * resolution;
                    pbr.z = br.representative->z;

                    add_triangle(ptl, ptr, pbl);
                    add_triangle(pbl, ptr, pbr);
                }
            }
        }


        std::shared_ptr<visualization_msgs::MarkerArray> markers = std::make_shared<visualization_msgs::MarkerArray>();
        markers->markers.push_back(floor);
        msg::publish(out_marker_, markers);
    }

private:
    Input* in_;
    Output* out_;
    Output* out_marker_;

    double max_floor_height_;
    double max_floor_angle_;
    double min_obstacle_angle_;

    int skip_;

    bool keep_organized_;
};


}

CSAPEX_REGISTER_CLASS(csapex::VelodyneSegmentation, csapex::Node)

