
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_handle.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <fstream>
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
                const std::size_t TIME = 0;
                const std::size_t UTM_X = 1;
                const std::size_t UTM_Y = 2;
                const std::size_t UTM_Z = 3;

                std::vector<std::string> csv_values;
                boost::split(csv_values, line, boost::is_any_of("\t "));

                double timestamp = str2double(csv_values[TIME]);

                ros::Time stamp;
                stamp.fromSec(timestamp);

                double x = str2double(csv_values[UTM_X]);
                double y = str2double(csv_values[UTM_Y]);
                double z = str2double(csv_values[UTM_Z]);

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

                double timestamp = str2double(csv_values[0]);

                ros::Time stamp;
                stamp.fromSec(timestamp);

                double yaw = str2double(csv_values[1]);
                double pitch = str2double(csv_values[2]);
                double roll = str2double(csv_values[3]);
                double x = str2double(csv_values[4]);
                double y = str2double(csv_values[5]);
                double z = str2double(csv_values[6]);

                tf::Transform pose(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x,y,z));

                tf::StampedTransform stamped_pose(pose, stamp, "odom", "base_link");
                poses_odom_.push_back(stamped_pose);
            }
            ++line_no;
        }
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
            }
        }

        if(!valid_) {
            node_handle_->setWarning("cannot import, something went wrong.");
            return;
        }

        msg::publish(out_gps_pose_, transform2Message(*next_gps_pose_));
        msg::publish(out_odom_pose_, transform2Message(*next_odom_pose_));

        ++next_gps_pose_;
        ++next_odom_pose_;

        if(next_gps_pose_ == poses_gps_.end()) {
            playing_ = false;
        }
        if(next_odom_pose_ == poses_odom_.end()) {
            playing_ = false;
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

    bool request_import_;
    bool valid_;
    bool playing_;

    std::string gps_file_;
    std::string odom_file_;
    std::string wheels_file_;

    std::vector<tf::StampedTransform> poses_gps_;
    std::vector<tf::StampedTransform> poses_odom_;

    std::vector<tf::StampedTransform>::iterator next_gps_pose_;
    std::vector<tf::StampedTransform>::iterator next_odom_pose_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::LAASImporter, csapex::Node)

