
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <person_msgs/Person.h>
#include <tf/tf.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class PersonToTransform : public Node
{
public:
    PersonToTransform()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<person_msgs::Person>("Person");
        out_ = modifier.addOutput<TransformMessage>("Transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        auto person = msg::getMessage<person_msgs::Person>(in_);
        auto res = std::make_shared<TransformMessage>(person->header.frame_id, "person");
        tf::poseMsgToTF(person->pose, res->value);
        msg::publish(out_, res);
    }

private:
    Input* in_;
    Output* out_;

};

}

CSAPEX_REGISTER_CLASS(csapex::PersonToTransform, csapex::Node)

