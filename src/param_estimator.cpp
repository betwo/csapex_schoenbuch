/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/generic_ros_message.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_core_plugins/timestamp_message.h>

/// SYSTEM
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

using namespace csapex::connection_types;


namespace csapex
{

class ParamEstimator : public Node
{
public:
    ParamEstimator()
        : x(0), y(0), theta(0), init_(false)
    {
        reset();
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_wheel_ = modifier.addInput<std_msgs::Float64MultiArray>("wheel odom");
        in_odom_= modifier.addMultiInput<GenericPointerMessage<nav_msgs::Odometry>, TimestampMessage>("odom or time stamp");

        out_ = modifier.addOutput<TransformMessage>("pose");
    }


    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("x_icr",   -0.1, 0.0, 0.0, 0.001), x_icr_);
        params.addParameter(param::ParameterFactory::declareRange("y_icr_l", 0.001, 0.75, 0.0, 0.001), y_icr_l);
        params.addParameter(param::ParameterFactory::declareRange("y_icr_r", -0.75, -0.001, 0.0, 0.001), y_icr_r);
        params.addParameter(param::ParameterFactory::declareRange("alpha_l", 0.5, 1.0, 1.0, 0.001), alpha_l);
        params.addParameter(param::ParameterFactory::declareRange("alpha_r", 0.5, 1.0, 1.0, 0.001), alpha_r);

        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*) {
            reset();
        });
    }

    void reset()
    {
        x = 0;
        y = 0;
        theta = 0;

        init_ = false;
    }

    void process()
    {
        TimestampMessage::Tp current_stamp;

        if(msg::isMessage<GenericPointerMessage<nav_msgs::Odometry>>(in_odom_)) {
            auto odom = msg::getMessage<nav_msgs::Odometry>(in_odom_);
            std::chrono::nanoseconds nano(odom->header.stamp.toNSec());
            current_stamp = TimestampMessage::Tp(std::chrono::duration_cast<std::chrono::microseconds>(nano));

        } else if(msg::isMessage<TimestampMessage>(in_odom_)) {
            auto time = msg::getMessage<TimestampMessage>(in_odom_);
            current_stamp = time->value;

        } else {
            throw std::runtime_error("invalid input type");
        }

        if(!init_) {
            last_stamp_ = current_stamp;
            init_ = true;
        }

        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(current_stamp - last_stamp_);
        last_stamp_ = current_stamp;
        double dt = msec.count() * 1e-6;

        auto wheel_odom = msg::getMessage<std_msgs::Float64MultiArray>(in_wheel_);

        double vl = 0.5 * (wheel_odom->data[0] + wheel_odom->data[3]);
        double vr = 0.5 * (wheel_odom->data[1] + wheel_odom->data[2]);

        double norm = 1.0 / (y_icr_r - y_icr_l);

        double vx = norm * (alpha_l * y_icr_r * vl  -  alpha_r * y_icr_l * vr);
        double vy = norm * (-alpha_l * x_icr_ * vl   +  alpha_r * x_icr_ * vr);
        double omega = norm * (alpha_l * vl - alpha_r * vr);

        double rvx = std::cos(theta) * vx - std::sin(theta) * vy;
        double rvy = std::sin(theta) * vx + std::cos(theta) * vy;

        x += rvx * dt;
        y += rvy * dt;
        theta += omega * dt;

        tf::Vector3 origin(x, y, 0);
        tf::Quaternion q = tf::createQuaternionFromYaw(theta);
        tf::Transform pose(q, origin);

        TransformMessage::Ptr result = std::make_shared<TransformMessage>("/map", "/base_link");
        result->value = pose;

        msg::publish(out_, result);
    }


    double x_icr_;
    double y_icr_l;
    double y_icr_r;
    double alpha_l;
    double alpha_r;

    double x;
    double y;
    double theta;

    bool init_;

    TimestampMessage::Tp last_stamp_;


private:
    Input* in_odom_;
    Input* in_wheel_;

    Output* out_;

};

}

CSAPEX_REGISTER_CLASS(csapex::ParamEstimator, csapex::Node)

