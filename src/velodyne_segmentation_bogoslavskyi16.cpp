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
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_ros/tf_listener.h>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}

class VelodyneSegmentationBogoslavskyi16 : public Node
{
public:
    VelodyneSegmentationBogoslavskyi16()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");

        out_ = modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareAngle("alpha", 0.0),
                            alpha_);
        params.addParameter(param::ParameterFactory::declareAngle("theta", 0.0),
                            theta_);

        params.addParameter(param::ParameterFactory::declareValue("min_cluster_size", 3), min_cluster_size_);
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        typename pcl::PointCloud<pcl::PointXYZI>::Ptr input = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(msg->value);
        if(input) {
            const pcl::PointCloud<pcl::PointXYZI>& cloud = *input;

            std::shared_ptr<std::vector<pcl::PointIndices> > cluster_indices = segment(cloud);

            msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, cluster_indices);
        }
    }

    std::shared_ptr<std::vector<pcl::PointIndices>> segment(const pcl::PointCloud<pcl::PointXYZI>& in)
    {
        pcl::PointCloud<pcl::PointXYZL> labels;
        pcl::copyPointCloud(in, labels);

        for(pcl::PointXYZL& pt : labels.points) {
            pt.label = 0;
        }

        std::shared_ptr<std::vector<pcl::PointIndices>> res = std::make_shared<std::vector<pcl::PointIndices>>();

        struct Pt {
            int row;
            int col;

            Pt()
              : row(-1), col(-1)
            {}

            Pt(int row, int col)
              : row(row), col(col)
            {}
        };


        int current_label = 0;

        int rows = in.height;
        int cols = in.width;

        apex_assert(rows > 1 && cols > 1);

        static const int nx[] = {-1, 0, 1, 0};
        static const int ny[] = {0, -1, 0, 1};

        for(int r = 0; r < rows; ++r) {
            for(int c = 0; c < cols; ++c) {
                pcl::PointXYZL& label = labels.at(c, r);
                if(label.label == 0) {
                    if(res->empty() || !res->back().indices.empty()) {
                        ++current_label;
                        res->emplace_back();
                    }
                    pcl::PointIndices& cluster = res->back();

                    std::deque<Pt> Q;
                    Q.push_back({r,c});
                    while(!Q.empty()) {
                        Pt top = Q.front();
                        Q.pop_front();

                        pcl::PointXYZL& pt = labels.at(top.col, top.row);
                        if(pt.label != 0) {
                            continue;
                        }

                        pt.label = current_label;

                        int idx = top.row * cols + top.col;
                        cluster.indices.push_back(idx);

                        double R = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);

                        for(int n = 0; n < 4; ++n){
                            Pt npt;
                            npt.row = top.row + ny[n];
                            if(npt.row < 0 || npt.row >= rows) {
                                continue;
                            }
                            npt.col = (top.col + nx[n] + cols) % cols;

                            pcl::PointXYZL& nlabel = labels.at(npt.col, npt.row);
                            if(nlabel.label != 0) {
                                continue;
                            }

                            double Rn = std::sqrt(nlabel.x*nlabel.x + nlabel.y*nlabel.y + nlabel.z*nlabel.z);

                            double d1 = std::max(R, Rn);
                            double d2 = std::min(R, Rn);

                            if(std::atan((d2 * std::sin(alpha_) / ((d1 - d2) * std::cos(alpha_)))) > theta_) {
                                Q.push_back(npt);
                            }
                        }
                    }

                    if((int)cluster.indices.size() < min_cluster_size_) {
                        cluster.indices.clear();
                    }
                }
            }
        }

        adebug << "found " << (current_label) << " clusters" << std::endl;

        return res;
    }

private:
    Input* in_;
    Output* out_;

    double alpha_;
    double theta_;

    int min_cluster_size_;
};


}

CSAPEX_REGISTER_CLASS(csapex::VelodyneSegmentationBogoslavskyi16, csapex::Node)
