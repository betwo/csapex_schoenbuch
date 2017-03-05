
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_handle.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/msg/end_of_sequence_message.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class LAASImporter : public Node
{
public:
    LAASImporter()
        : request_import_(false),
          valid_(false),
          playing_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        out_gps_pose_ = modifier.addOutput<TransformMessage>("GPS Pose");
        out_odom_pose_ = modifier.addOutput<TransformMessage>("Odom Pose");
        out_wheels_ = modifier.addOutput<std_msgs::Float64MultiArray>("Wheel Velocities");

        out_time_ = modifier.addOutput<TimestampMessage>("Timestamp");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareFileInputPath("gps", "", "*.log"), gps_file_);
        params.addParameter(param::ParameterFactory::declareFileInputPath("odometry", "", "*.log"), odom_file_);
        params.addParameter(param::ParameterFactory::declareFileInputPath("wheels", "", "*.log"), wheels_file_);

        params.addParameter(param::ParameterFactory::declareTrigger("start"), [this](param::Parameter*) {
            request_import_ = true;
            yield();
        });

        params.addParameter(param::ParameterFactory::declareRange("frame", 0, 1, 0, 1), [this](param::Parameter* p)
        {

        });

        params.addParameter(param::ParameterFactory::declareInterval("range", 0, 1, 0, 1, 1), [this](param::Parameter* p)
        {
            updateRange();
        });
    }

    void updateRange()
    {
        param::IntervalParameterPtr range = getParameter<param::IntervalParameter>("range");
        apex_assert(range);
        last_wheels_ = wheel_velocities_.begin();
        std::advance(last_wheels_, range->upper<int>());

        yield();
    }

    void import()
    {
        bf3::path gps(gps_file_);
        if(!bf3::exists(gps)) {
            throw std::runtime_error(std::string("Cannot load the file ") + gps_file_);
        }
        bf3::path odom(odom_file_);
        if(!bf3::exists(odom)) {
            throw std::runtime_error(std::string("Cannot load the file ") + odom_file_);
        }
        bf3::path wheel(wheels_file_);
        if(!bf3::exists(wheel)) {
            throw std::runtime_error(std::string("Cannot load the file ") + wheels_file_);
        }

        std::ifstream gps_stream(gps.string());
        importGPS(gps_stream);

        std::ifstream odom_stream(odom.string());
        importOdometry(odom_stream);

        std::ifstream wheel_stream(wheel.string());
        importWheels(wheel_stream);

        valid_ = true;
    }


    void importGPS(std::ifstream& gps_file_stream)
    {
        poses_gps_.clear();

        std::string line;
        int line_no = 0;
        while (std::getline(gps_file_stream, line))
        {
            if(line_no > 0) {

                enum class Gps
                {
                    time, robot_utm_x, robot_utm_y, robot_utm_z, x_sd, y_sd, z_sd, mode, solStatus,
                    posType, longitudZoneNumber, latitudinalZoneLetter, sensor_northing, sensor_easting, sensor_height, undulation, datumID,
                    baseStationID0, baseStationID1, baseStationID2, baseStationID3,
                    differentialAge, solutionAge, nbSatTracked, nbSatTracked2, nbGPSL1Used, nbGPSL1, nbGPSL2,
                    reserved0, reserved1, reserved2, reserved3, solStatus2, velocityType, latency, age, horizontalSpeed, actualDirection, verticalSpeed
               };

                std::vector<std::string> csv_values;
                boost::split(csv_values, line, boost::is_any_of("\t "));

                double timestamp = str2double(csv_values[(int) Gps::time]);

                ros::Time stamp;
                stamp.fromSec(timestamp);

                double x = str2double(csv_values[(int) Gps::robot_utm_x]);
                double y = str2double(csv_values[(int) Gps::robot_utm_y]);
                double z = str2double(csv_values[(int) Gps::robot_utm_z]);

                double theta = 0;

                tf::Transform pose(tf::createQuaternionFromYaw(theta), tf::Vector3(x,y,z));

                tf::StampedTransform stamped_pose(pose, stamp, "gps", "base_link");
                poses_gps_.push_back(stamped_pose);
            }
            ++line_no;
        }
    }

    void importOdometry(std::ifstream& odometry_file_stream)
    {
        poses_odom_.clear();

        std::string line;
        int line_no = 0;
        while (std::getline(odometry_file_stream, line))
        {
            if(line_no > 0) {
                std::vector<std::string> csv_values;
                boost::split(csv_values, line, boost::is_any_of("\t "));

                enum class Odom
                {
                    date, yaw, pitch, roll, x, y, z
                };

                double timestamp = str2double(csv_values[(int) Odom::date]);

                ros::Time stamp;
                stamp.fromSec(timestamp);

                double yaw = str2double(csv_values[(int) Odom::yaw]);
                double pitch = str2double(csv_values[(int) Odom::pitch]);
                double roll = str2double(csv_values[(int) Odom::roll]);
                double x = str2double(csv_values[(int) Odom::x]);
                double y = str2double(csv_values[(int) Odom::y]);
                double z = str2double(csv_values[(int) Odom::z]);

                tf::Transform pose(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x,y,z));

                tf::StampedTransform stamped_pose(pose, stamp, "odom", "base_link");
                poses_odom_.push_back(stamped_pose);
            }
            ++line_no;
        }
    }

    void importWheels(std::ifstream& wheel_file_stream)
    {
        wheel_velocities_.clear();

        std::string line;
        int line_no = 0;
        while (std::getline(wheel_file_stream, line))
        {
            if(line_no > 0) {
                std::vector<std::string> csv_values;
                boost::split(csv_values, line, boost::is_any_of("\t "));

                enum class FullLog
                {
                    date, date1, date2, gyroAngle, gyroRate, sent_velocity_reference, sent_velocity_command, sent_turn_reference, sent_turn_command,
                    straight, straight_angle, fault_status0, fault_status1, faultstatus2, fault_status3, mcu_fault0, mcu_fault1, mcu_fault2, mcu_fault3,
                    frame_count, operational_state, dynamic_response, min_propulstion_bat_soc, aux_batt_soc,
                    inertial_x_acc, inertial_y_acc, inertial_x_rate, inertial_y_rate, inertial_z_rate,
                    pse_pitch, pse_pitch_rate, pse_roll, pse_roll_rate, pse_yaw_rate, pse_data_is_valid,
                    yaw_rate_limit, vel_limit, linear_accel, linear_vel, differential_wheel_vel,
                    right_front_vel, left_front_vel, right_rear_vel, left_rear_vel,
                    right_front_pos, left_front_pos, right_rear_pos, left_rear_pos,
                    linear_pos, right_front_current, left_front_current, right_rear_current, left_rear_current,
                    max_motor_current, right_front_current_limit, left_front_current_limit, right_rear_current_limit, left_rear_current_limit,
                    min_motor_current_limit, front_base_batt_1_soc, front_base_batt_2_soc, rear_base_batt_1_soc, rear_base_batt_2_soc,
                    front_base_batt_1_temp, front_base_batt_2_temp, rear_base_batt_1_temp, rear_base_batt_2_temp, vel_target, yaw_rate_target,
                    angle_target, aux_batt_voltage, aux_batt_current, aux_batt_temp, abb_system_status, abb_batt_status, aux_batt_faults,
                    ccu_7p2_battery_voltage, sp_sw_build_id, uip_sw_build_id, mcu_inst_power0, mcu_inst_power1, mcu_inst_power2, mcu_inst_power3,
                    mcu_total_energy0, mcu_total_energy1, mcu_total_energy2, mcu_total_energy3
                };


                double timestamp = str2double(csv_values[(int) FullLog::date]);

                std::chrono::microseconds stamp(static_cast<int64_t>(timestamp * 1e6));

                double fl = str2double(csv_values[(int) FullLog::left_front_vel]);
                double fr = str2double(csv_values[(int) FullLog::right_front_vel]);
                double rl = str2double(csv_values[(int) FullLog::left_rear_vel]);
                double rr = str2double(csv_values[(int) FullLog::right_rear_vel]);

                std::shared_ptr<std_msgs::Float64MultiArray> msg(new std_msgs::Float64MultiArray);

                msg->data.push_back(fl);
                msg->data.push_back(fr);
                msg->data.push_back(rr);
                msg->data.push_back(rl);

                wheel_velocities_.push_back(std::make_pair(stamp, msg));
            }
            ++line_no;
        }

        param::RangeParameterPtr frame = getParameter<param::RangeParameter>("frame");
        frame->setMax<int>(wheel_velocities_.size());

        param::IntervalParameterPtr range = getParameter<param::IntervalParameter>("range");
        range->setMax<int>(wheel_velocities_.size());

        updateRange();

    }

    bool canProcess() const override
    {
        return request_import_ || playing_;
    }

    void process() override
    {

        if(request_import_) {
            request_import_ = false;
            import();

            if(valid_) {
                playing_ = true;

                next_gps_pose_ = poses_gps_.begin();
                next_odom_pose_ = poses_odom_.begin();
                next_wheels_ = wheel_velocities_.begin();

                param::IntervalParameterPtr range = getParameter<param::IntervalParameter>("range");
                std::advance(next_wheels_, range->lower<int>());
            }
        }

        if(!valid_) {
            node_handle_->setWarning("cannot import, something went wrong.");
            return;
        }

        // wheel velocities are used as anchor, so they must be available
        if(next_wheels_ >= last_wheels_) {
            msg::publish(out_gps_pose_, connection_types::makeEmpty<connection_types::EndOfSequenceMessage>());
            msg::publish(out_odom_pose_, connection_types::makeEmpty<connection_types::EndOfSequenceMessage>());
            msg::publish(out_wheels_, connection_types::makeEmpty<connection_types::EndOfSequenceMessage>());
            msg::publish(out_time_, connection_types::makeEmpty<connection_types::EndOfSequenceMessage>());

            playing_ = false;

        } else  {
            // find the stamp for data to publish in the current wheel velocities
            std::chrono::microseconds micro = next_wheels_->first;
            ros::Time stamp;
            stamp.fromNSec(micro.count() * 1e3);

            // send time and wheel velocities
            TimestampMessage::Ptr time_msg(new TimestampMessage(TimestampMessage::Tp(micro)));
            msg::publish(out_time_, time_msg);
            msg::publish(out_wheels_, next_wheels_->second);

            ++next_wheels_;

            setParameter<int>("frame", std::distance(wheel_velocities_.begin(), next_wheels_));

            // find corresponding gps pose
            while((next_gps_pose_+1) != poses_gps_.end() &&
                  (next_gps_pose_+1)->stamp_ <= stamp) {
                ++next_gps_pose_;
            }

            // find corresponding odom pose
            while((next_odom_pose_+1) != poses_odom_.end() &&
                  (next_odom_pose_+1)->stamp_ <= stamp) {
                ++next_odom_pose_;
            }

            if(next_gps_pose_ < poses_gps_.end()) {
                msg::publish(out_gps_pose_, transform2Message(*next_gps_pose_));
            }

            if(next_gps_pose_ < poses_gps_.end()) {
                msg::publish(out_odom_pose_, transform2Message(*next_odom_pose_));
            }
        }

    }


    double str2double(const std::string& str)  const
    {
        double tmp;
        std::stringstream conv(str);
        conv >> tmp;
        return tmp;
    }

    TransformMessage::Ptr transform2Message(const tf::StampedTransform& pose)
    {
        TransformMessage::Ptr result = std::make_shared<TransformMessage>(pose.frame_id_, pose.child_frame_id_);
        result->value = pose;
        result->frame_id = pose.frame_id_;
        result->child_frame = pose.child_frame_id_;
        result->stamp_micro_seconds = pose.stamp_.toNSec() * 1e-3;

        return result;
    }


private:
    Output* out_gps_pose_;
    Output* out_odom_pose_;
    Output* out_wheels_;
    Output* out_time_;

    bool request_import_;
    bool valid_;
    bool playing_;

    std::string gps_file_;
    std::string odom_file_;
    std::string wheels_file_;

    std::vector<tf::StampedTransform> poses_gps_;
    std::vector<tf::StampedTransform> poses_odom_;
    std::vector<std::pair<std::chrono::microseconds, std::shared_ptr<std_msgs::Float64MultiArray>>> wheel_velocities_;

    std::vector<tf::StampedTransform>::iterator next_gps_pose_;
    std::vector<tf::StampedTransform>::iterator next_odom_pose_;
    std::vector<std::pair<std::chrono::microseconds, std::shared_ptr<std_msgs::Float64MultiArray>>>::iterator next_wheels_;
    std::vector<std::pair<std::chrono::microseconds, std::shared_ptr<std_msgs::Float64MultiArray>>>::iterator last_wheels_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::LAASImporter, csapex::Node)

