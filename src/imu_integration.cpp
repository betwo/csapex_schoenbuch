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
        params.addParameter(param::ParameterFactory::declareRange("drift/yaw", -1.10, 1.10, 0.00, 0.00001), drift_yaw_);

//        params.addParameter(param::ParameterFactory::declareRange("acc/gravity", 9.5, 10., 9.81, 0.00001), gravity_);
//        params.addParameter(param::ParameterFactory::declareRange("acc/decay", 0.0, 1.0, 1.0, 0.00001), acc_decay_);
//        params.addParameter(param::ParameterFactory::declareRange("acc/queue", 1, 32, 8, 1), acceleration_queue_length_);

    }

    void reset()
    {
        world_T_base_link = tf::Pose(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));
        last_imu.reset();
        last_odom.reset();

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

//        tf::Transform world_T_imu = world_T_base_link * base_link_T_imu;

        tf::Quaternion rotation_imu_raw;
        tf::quaternionMsgToTF(imu->orientation, rotation_imu_raw);

        tf::Quaternion rotation_imu = rotation_imu_raw * base_link_T_imu.getRotation();

        tf::Matrix3x3 rotation_mat(rotation_imu);
        double y,p,r;
        rotation_mat.getEulerYPR(y,p,r);

//        p *= -1.0;
//        r *= -1.0;

        p += offset_pitch_;
        r += offset_roll_;

        //y = tf::getYaw(world_T_imu.getRotation());
        y = tf::getYaw(world_T_base_link.getRotation());
//        y = tf::getYaw(current_pose_odom.getRotation());

        rotation_mat.setEulerYPR(y,p,r);
        tf::Quaternion rotation_base_link;
        rotation_mat.getRotation(rotation_base_link);

//        rotation_base_link = rotation_imu;

        tf::Vector3 acc_imu;
        tf::vector3MsgToTF(imu->linear_acceleration, acc_imu);

        if(last_imu) {
            ros::Duration dt_imu = imu->header.stamp - last_imu->header.stamp;
            ros::Duration dt_odom = odom->header.stamp - last_odom->header.stamp;
            world_T_base_link.setRotation(rotation_base_link);

            tf::Transform delta_base_link = last_pose_odom.inverse() * current_pose_odom;

            //delta_base_link.setRotation(tf::createQuaternionFromYaw(drift_yaw_) * delta_base_link.getRotation());

            tf::Transform drift(tf::createQuaternionFromYaw(drift_yaw_ * dt_odom.toSec()), tf::Vector3(0,0,0));
            delta_base_link = drift * delta_base_link;

//            tf::Vector3 acc_base_link = base_link_T_imu * acc_imu;
//            ainfo << "acc raw: " << acc_base_link.x() << ", " << acc_base_link.y() << ", " << acc_base_link.z() << ", dt: " << dt << ", g: " << gravity_ << std::endl;

//            accu_accel_.push_back(acc_base_link.z());

//            while(accu_accel_.size() > acceleration_queue_length_) {
//                accu_accel_.pop_front();
//            }

//            double acc_z = std::accumulate(accu_accel_.begin(), accu_accel_.end(), 0.0) / (double) accu_accel_.size();
//            acc_z -= gravity_;
//            ainfo << "acc: " << acc_z << std::endl;
//            acc_base_link.setZ(acc_z);

//            acc_base_link.setX(0);
//            acc_base_link.setY(0);


//            v_base_link *= acc_decay_;
//            v_base_link += acc_base_link * dt.toSec();

            tf::Transform external_offset(tf::createIdentityQuaternion(), v_base_link * dt_odom.toSec());
            delta_base_link = delta_base_link * external_offset;

            world_T_base_link = world_T_base_link * delta_base_link;

        } else {
            v_base_link = tf::Vector3(0,0,0);
            world_T_base_link.setRotation(rotation_base_link);

            accu_accel_.clear();
        }

//        world_T_base_link.setRotation(rotation_base_link);
//        world_T_base_link.setRotation(tf::createQuaternionFromYaw(drift_yaw_) * world_T_base_link.getRotation());

        last_imu = imu;
        last_odom = odom;
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
    std::shared_ptr<nav_msgs::Odometry const> last_odom;
    tf::Transform last_pose_odom;

    tf::Pose world_T_base_link;

    tf::Vector3 v_base_link;

    std::deque<double> accu_accel_;

    double offset_roll_;
    double offset_pitch_;
    double drift_yaw_;

//    int acceleration_queue_length_;
//    double acc_decay_;

//    double gravity_;
};

}

CSAPEX_REGISTER_CLASS(csapex::IMUIntegration, csapex::Node)

