/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>

/// SYSTEM
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace csapex::connection_types;


namespace csapex
{

class IMUIntegration : public Node
{
public:
    IMUIntegration()
    {
        reset();
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_imu_= modifier.addInput<sensor_msgs::Imu>("IMU");
        in_odom_= modifier.addInput<nav_msgs::Odometry>("odom");
        in_tf_imu_to_base_link_ = modifier.addInput<TransformMessage>("IMU <T> base link");

        out_pose_ = modifier.addOutput<TransformMessage>("Pose");
        out_imu_ = modifier.addOutput<TransformMessage>("Imu TF");
    }


    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*) {
            reset();
        });

        params.addParameter(param::ParameterFactory::declareAngle("offset/roll", 0.00), offset_roll_);
        params.addParameter(param::ParameterFactory::declareAngle("offset/pitch", 0.00), offset_pitch_);
        params.addParameter(param::ParameterFactory::declareRange("drift/yaw", -1.10, 1.10, 0.00, 0.0001), drift_yaw_);
    }

    void reset()
    {
        world_T_base_link = tf::Pose(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));
        last_imu.reset();

        last_pose_odom = tf::Pose(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));
    }

    void process()
    {
        std::shared_ptr<sensor_msgs::Imu const> imu = msg::getMessage<sensor_msgs::Imu>(in_imu_);
        std::shared_ptr<nav_msgs::Odometry const> odom = msg::getMessage<nav_msgs::Odometry>(in_odom_);

        auto trafo_msg = msg::getMessage<TransformMessage>(in_tf_imu_to_base_link_);

        tf::Transform imu_T_base_link = trafo_msg->value;
        tf::Transform base_link_T_imu = imu_T_base_link.inverse();

        tf::Transform current_pose_odom;
        tf::poseMsgToTF(odom->pose.pose, current_pose_odom);

        tf::Transform world_T_imu = world_T_base_link * base_link_T_imu;

        tf::Quaternion rotation_imu;
        tf::quaternionMsgToTF(imu->orientation, rotation_imu);

        tf::Matrix3x3 rotation_mat(rotation_imu);
        double y,p,r;
        rotation_mat.getEulerYPR(y,p,r);

        p *= -1.0;
        r *= -1.0;

        p += offset_pitch_;
        r += offset_roll_;

        //y = tf::getYaw(world_T_imu.getRotation());
        y = tf::getYaw(world_T_base_link.getRotation());
//        y = tf::getYaw(current_pose_odom.getRotation());

        rotation_mat.setEulerYPR(y,p,r);
        tf::Quaternion rotation_base_link;
        rotation_mat.getRotation(rotation_base_link);

        if(last_imu) {
//            ros::Duration dt = imu->header.stamp - last_imu->header.stamp;
            world_T_base_link.setRotation(rotation_base_link);

            tf::Transform delta_base_link = last_pose_odom.inverse() * current_pose_odom;

            //delta_base_link.setRotation(tf::createQuaternionFromYaw(drift_yaw_) * delta_base_link.getRotation());

            tf::Transform drift(tf::createQuaternionFromYaw(drift_yaw_), tf::Vector3(0,0,0));
            delta_base_link = drift * delta_base_link;

            world_T_base_link = world_T_base_link * delta_base_link;

        } else {
            world_T_base_link.setRotation(rotation_base_link);
        }

//        world_T_base_link.setRotation(rotation_base_link);
//        world_T_base_link.setRotation(tf::createQuaternionFromYaw(drift_yaw_) * world_T_base_link.getRotation());

        last_imu = imu;
        last_pose_odom = current_pose_odom;




        tf::StampedTransform pose_stamped(world_T_base_link, imu->header.stamp, "/world", "/base_link");

        TransformMessage::Ptr trafo(new TransformMessage(pose_stamped.frame_id_, pose_stamped.child_frame_id_));
        trafo->value = pose_stamped;
        trafo->stamp_micro_seconds = imu->header.stamp.toNSec() * 1e-3;

        msg::publish(out_pose_, trafo);



        tf::StampedTransform imu_stamped(tf::Transform(rotation_imu, tf::Vector3(0,0,0)), imu->header.stamp, "/imu_link", "/imu");

        TransformMessage::Ptr trafo_imu(new TransformMessage(imu_stamped.frame_id_, imu_stamped.child_frame_id_));
        trafo_imu->value = imu_stamped;
        trafo_imu->stamp_micro_seconds = imu->header.stamp.toNSec() * 1e-3;

        msg::publish(out_imu_, trafo_imu);
    }


private:
    Input* in_imu_;
    Input* in_odom_;
    Input* in_tf_imu_to_base_link_;
    Output* out_pose_;
    Output* out_imu_;

    std::shared_ptr<sensor_msgs::Imu const> last_imu;
    tf::Transform last_pose_odom;

    tf::Pose world_T_base_link;

    double offset_roll_;
    double offset_pitch_;
    double drift_yaw_;
};

}

CSAPEX_REGISTER_CLASS(csapex::IMUIntegration, csapex::Node)

