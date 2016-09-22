/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/msg/generic_value_message.hpp>
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
    }

    void shutdown()
    {
    }

};

}

CSAPEX_REGISTER_CLASS(csapex::RegisterSchoenbuch, csapex::CorePlugin)
