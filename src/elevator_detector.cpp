
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/signal/event.h>

/// SYSTEM
#include <pcl/common/common.h>
#include <boost/optional.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class ElevatorDetector : public Node
{
public:
    ElevatorDetector()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");

        event_enter_ = modifier.addEvent("Enter");
        event_leave_ = modifier.addEvent("Leave");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("threshold", 0.0, 5.0, 4.0, 0.01),
                            threshold_);
    }

    void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        typename pcl::PointCloud<pcl::PointXYZI>::Ptr input = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(msg->value);
        if(input) {
            const pcl::PointCloud<pcl::PointXYZI>& cloud = *input;

            double max_dist = 0;
            for(const pcl::PointXYZI& pt : cloud.points) {
                max_dist = std::max<double>(max_dist, std::hypot(pt.x, pt.y));
            }

            bool elev = (max_dist < threshold_);

            if(!inside_ || elev != inside_.get()) {
                inside_ = elev;
                if(elev) {
                    event_enter_->trigger();
                } else {
                    event_leave_->trigger();
                }
            }
        }
    }

private:
    Input* in_;

    double threshold_;
    boost::optional<bool> inside_;

    Event* event_enter_;
    Event* event_leave_;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::ElevatorDetector, csapex::Node)

