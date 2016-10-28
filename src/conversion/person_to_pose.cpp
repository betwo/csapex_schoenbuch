
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <person_msgs/Person.h>
#include<geometry_msgs/Pose.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class PersonToPose : public Node
{
public:
    PersonToPose()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<person_msgs::Person>("Person");
        out_ = modifier.addOutput<geometry_msgs::Pose>("Pose");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        auto person = msg::getMessage<person_msgs::Person>(in_);
        auto res = std::make_shared<geometry_msgs::Pose>(person->pose);
        msg::publish(out_, res);
    }

private:
    Input* in_;
    Output* out_;

};

}

CSAPEX_REGISTER_CLASS(csapex::PersonToPose, csapex::Node)

