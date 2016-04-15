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
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        typename pcl::PointCloud<pcl::PointXYZI>::Ptr input = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(msg->value);
        if(input) {
            const pcl::PointCloud<pcl::PointXYZI>& cloud = *input;
            pcl::PointCloud<pcl::PointXYZI>::Ptr res(new pcl::PointCloud<pcl::PointXYZI>);
            res->header = input->header;

            segment(cloud, *res);

            PointCloudMessage::Ptr output_msg(new PointCloudMessage(cloud.header.frame_id, cloud.header.stamp));
            output_msg->value = res;

            msg::publish(out_, output_msg);
        }
    }

    void segment(const pcl::PointCloud<pcl::PointXYZI>& in, pcl::PointCloud<pcl::PointXYZI>& out)
    {
        double dimensions = 10.0 /*m*/;
        double resolution = 0.25;
        int size = std::ceil(dimensions / resolution);
        int half_size = size / 2;

        struct Bin {
            std::vector<pcl::PointXYZI> pts;
            pcl::PointXYZI* representative = nullptr;
        };

        std::vector<Bin> bins(size * size);


        for(std::size_t idx = 0, n = in.size(); idx < n; ++idx) {
            const pcl::PointXYZI& pt = in.points.at(idx);
            int col = pt.x * resolution + half_size;
            int row = pt.y * resolution + half_size;
            if(col >= 0 && col < size && row >= 0 && row < size) {
                Bin& bin = bins.at(row * size + col);
                bin.pts.push_back(pt);
            }
        }

        Bin* next_bin = &bins.front();
        for(int row = 0; row < size; ++row) {
            for(int col = 0; col < size; ++col, ++next_bin) {
                Bin& bin = *next_bin;
                if(bin.pts.size() > 3) {
                    std::sort(bin.pts.begin(), bin.pts.end(),
                              [](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
                        return a.z < b.z;
                    });

                    bin.representative = &bin.pts.front();
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

        for(int row = 1; row < size; ++row) {
            for(int col = 1; col < size; ++col, ++next_bin) {
                Bin& tl = bins.at((row-1) * size + (col-1));
                Bin& tr = bins.at((row-1) * size + (col));
                Bin& bl = bins.at((row) * size + (col-1));
                Bin& br = bins.at((row) * size + (col));

                if(tl.representative && tr.representative && bl.representative && br.representative) {
                    geometry_msgs::Point ptl;
                    ptl.x = (col-1 - half_size) / resolution;
                    ptl.y = (row-1 - half_size) / resolution;
                    ptl.x = tl.representative->z;
                    floor.points.push_back(ptl);

                    geometry_msgs::Point ptr;
                    ptr.x = (col - half_size) / resolution;
                    ptr.y = (row-1 - half_size) / resolution;
                    ptr.x = tr.representative->z;
                    floor.points.push_back(ptr);

                    geometry_msgs::Point pbl;
                    pbl.x = (col-1 - half_size) / resolution;
                    pbl.y = (row - half_size) / resolution;
                    pbl.x = bl.representative->z;
                    floor.points.push_back(pbl);

                    geometry_msgs::Point pbr;
                    pbr.x = (col - half_size) / resolution;
                    pbr.y = (row - half_size) / resolution;
                    pbr.x = br.representative->z;
                    floor.points.push_back(pbr);
                }
            }
        }


        std::shared_ptr<visualization_msgs::MarkerArray> markers = std::make_shared<visualization_msgs::MarkerArray>();
        msg::publish(out_marker_, markers);
    }

private:
    Input* in_;
    Output* out_;
    Output* out_marker_;
};


}

CSAPEX_REGISTER_CLASS(csapex::VelodyneSegmentation, csapex::Node)

