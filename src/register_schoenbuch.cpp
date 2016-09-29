/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <person_msgs/Person.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN RegisterSchoenbuch : public CorePlugin
{
public:
    RegisterSchoenbuch()
    {
    }

    void init(CsApexCore& core)
    {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, person_msgs::Person>();
        MessageSerializer::registerDirectMessage<connection_types::GenericPointerMessage, person_msgs::Person>();

        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, person_msgs::Person>::registerConversion();
    }

    void shutdown()
    {
    }

};

}

CSAPEX_REGISTER_CLASS(csapex::RegisterSchoenbuch, csapex::CorePlugin)
