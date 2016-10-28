
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class ImuToTransform : public Node
{
public:
    ImuToTransform()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<sensor_msgs::Imu>("IMU");
        out_ = modifier.addOutput<TransformMessage>("Transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        auto imu = msg::getMessage<sensor_msgs::Imu>(in_);

        std::shared_ptr<TransformMessage> tf_out = std::make_shared<TransformMessage>(imu->header.frame_id, "/imu");
        tf::Quaternion q;
        tf::quaternionMsgToTF(imu->orientation, q);
        tf_out->value = tf::Transform(q, tf::Vector3(0,0,0));

        msg::publish(out_, tf_out);
    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::ImuToTransform, csapex::Node)

