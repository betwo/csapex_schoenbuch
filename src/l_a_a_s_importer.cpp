
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
        params.addParameter(param::ParameterFactory::declareBool("import_gps", false), import_gps_);
        params.addConditionalParameter(param::ParameterFactory::declareFileInputPath("gps", "", "*.log"), import_gps_, gps_file_);
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
        bf3::path odom(odom_file_);
        if(!bf3::exists(odom)) {
            throw std::runtime_error(std::string("Cannot load the file ") + odom_file_);
        }
        bf3::path wheel(wheels_file_);
        if(!bf3::exists(wheel)) {
            throw std::runtime_error(std::string("Cannot load the file ") + wheels_file_);
        }
        if(import_gps_) {
            bf3::path gps(gps_file_);
            if(!bf3::exists(gps)) {
                throw std::runtime_error(std::string("Cannot load the file ") + gps_file_);
            }
        }

        std::ifstream odom_stream(odom.string());
        importOdometry(odom_stream);

        std::ifstream wheel_stream(wheel.string());
        importWheels(wheel_stream);

        if(import_gps_) {
            bf3::path gps(gps_file_);
            std::ifstream gps_stream(gps.string());
            importGPS(gps_stream);
        }

        valid_ = true;
    }


    void importGPS(std::ifstream& gps_file_stream)
    {
        poses_gps_.clear();

        std::string line;
        int line_no = 0;

        std::vector<std::string> necessary_keys {"time", "robot_utm_x", "robot_utm_y", "robot_utm_z"};
        std::map<std::string, int> key_2_index_;
        for(const std::string& key : necessary_keys) {
            key_2_index_[key] = -1;
        }

        while (std::getline(gps_file_stream, line))
        {
            while(line.at(0) == '#') {
                line = line.substr(1);
            }
            std::vector<std::string> csv_values;
            boost::split(csv_values, line, boost::is_any_of("\t "));

            if(line_no == 0) {
                int index = 0;
                for(const std::string& header : csv_values) {
                    auto it = key_2_index_.find(header);
                    if(it != key_2_index_.end()) {
                        key_2_index_[header] = index;
                    }
                    ++index;
                }

                for(const auto& pair : key_2_index_) {
                    apex_assert_msg(pair.second != -1, std::string("the log file does not contain an entry called '") + pair.first + "'");
                }

            } else {
                double timestamp = str2double(csv_values.at(key_2_index_["time"]));

                ros::Time stamp;
                stamp.fromSec(timestamp);

                double x = str2double(csv_values.at(key_2_index_["robot_utm_x"]));
                double y = str2double(csv_values.at(key_2_index_["robot_utm_y"]));
                double z = str2double(csv_values.at(key_2_index_["robot_utm_z"]));

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

        std::vector<std::string> necessary_keys {"date", "yaw", "pitch", "roll", "x", "y", "z"};
        std::map<std::string, int> key_2_index_;
        for(const std::string& key : necessary_keys) {
            key_2_index_[key] = -1;
        }

        while (std::getline(odometry_file_stream, line))
        {
            while(line.at(0) == '#') {
                line = line.substr(1);
            }
            std::vector<std::string> csv_values;
            boost::split(csv_values, line, boost::is_any_of("\t "));

            if(line_no == 0) {
                int index = 0;
                for(const std::string& header : csv_values) {
                    auto it = key_2_index_.find(header);
                    if(it != key_2_index_.end()) {
                        key_2_index_[header] = index;
                    }
                    ++index;
                }

                for(const auto& pair : key_2_index_) {
                    apex_assert_msg(pair.second != -1, std::string("the log file does not contain an entry called '") + pair.first + "'");
                }

            } else {
                double timestamp = str2double(csv_values.at(key_2_index_["date"]));

                ros::Time stamp;
                stamp.fromSec(timestamp);

                double yaw = str2double(csv_values.at(key_2_index_["yaw"]));
                double pitch = str2double(csv_values.at(key_2_index_["pitch"]));
                double roll = str2double(csv_values.at(key_2_index_["roll"]));
                double x = str2double(csv_values.at(key_2_index_["x"]));
                double y = str2double(csv_values.at(key_2_index_["y"]));
                double z = str2double(csv_values.at(key_2_index_["z"]));

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

        std::vector<std::string> necessary_keys {"date", "right_front_vel", "left_front_vel", "left_rear_vel", "right_rear_vel"};
        std::map<std::string, int> key_2_index_;
        for(const std::string& key : necessary_keys) {
            key_2_index_[key] = -1;
        }

        while (std::getline(wheel_file_stream, line))
        {
            while(line.at(0) == '#') {
                line = line.substr(1);
            }
            std::vector<std::string> csv_values;
            boost::split(csv_values, line, boost::is_any_of("\t "));

            if(line_no == 0) {
                int index = 0;
                for(const std::string& header : csv_values) {
                    auto it = key_2_index_.find(header);
                    if(it != key_2_index_.end()) {
                        key_2_index_[header] = index;
                    }
                    ++index;
                }

                for(const auto& pair : key_2_index_) {
                    apex_assert_msg(pair.second != -1, std::string("the log file does not contain an entry called '") + pair.first + "'");
                }

            } else {
                double timestamp = str2double(csv_values.at(key_2_index_["date"]));

                std::chrono::microseconds stamp(static_cast<int64_t>(timestamp * 1e6));

                double fl = str2double(csv_values.at(key_2_index_["left_front_vel"]));
                double fr = str2double(csv_values.at(key_2_index_["right_front_vel"]));
                double rl = str2double(csv_values.at(key_2_index_["left_rear_vel"]));
                double rr = str2double(csv_values.at(key_2_index_["right_rear_vel"]));

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
            if(import_gps_) {
                msg::publish(out_gps_pose_, connection_types::makeEmpty<connection_types::EndOfSequenceMessage>());
            }
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
            if(import_gps_) {
                while((next_gps_pose_+1) != poses_gps_.end() &&
                      (next_gps_pose_+1)->stamp_ <= stamp) {
                    ++next_gps_pose_;
                }
            }

            // find corresponding odom pose
            while((next_odom_pose_+1) != poses_odom_.end() &&
                  (next_odom_pose_+1)->stamp_ <= stamp) {
                ++next_odom_pose_;
            }

            if(next_gps_pose_ < poses_gps_.end()) {
                msg::publish(out_gps_pose_, transform2Message(*next_gps_pose_));
            }

            if(next_odom_pose_ < poses_odom_.end()) {
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

    bool import_gps_;

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

