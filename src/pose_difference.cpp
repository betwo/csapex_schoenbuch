/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

using namespace csapex::connection_types;


namespace csapex
{

class PoseDifference : public Node
{
public:
    PoseDifference()
        : e(0)
    {
        reset();
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_wheel_ = modifier.addInput<TransformMessage>("wheel odom");
        in_gt_= modifier.addInput<TransformMessage>("ground truth");

        out_current_ = modifier.addOutput<double>("current error");
        out_accumulated_ = modifier.addOutput<double>("accumulated error");
    }


    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*) {
            reset();
        });
    }

    void reset()
    {
        e = 0;
    }

    void process()
    {
        auto gt_msg = msg::getMessage<TransformMessage>(in_gt_);
        auto wheel_msg = msg::getMessage<TransformMessage>(in_wheel_);

        tf::Transform gt = gt_msg->value;
        tf::Transform wheel = wheel_msg->value;

        auto delta_pos = gt.getOrigin() - wheel.getOrigin();

        double yaw_gt = tf::getYaw(gt.getRotation());
        double yaw_wheel = tf::getYaw(wheel.getRotation());

        double delta_rot = std::atan2(std::sin(yaw_gt) + std::sin(yaw_wheel),
                                      std::cos(yaw_gt) + std::cos(yaw_wheel));

        double de = delta_pos.dot(delta_pos) + delta_rot * delta_rot;

        e += de;

        msg::publish(out_current_, de);
        msg::publish(out_accumulated_, e);
    }


private:
    Input* in_gt_;
    Input* in_wheel_;

    Output* out_current_;
    Output* out_accumulated_;

    double e;
};

}

CSAPEX_REGISTER_CLASS(csapex::PoseDifference, csapex::Node)

