
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/generic_ros_message.h>
#include <csapex/signal/event.h>

/// SYSTEM
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class GenerateExplorationMap : public Node
{
public:
    GenerateExplorationMap()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_map_ = modifier.addInput<nav_msgs::OccupancyGrid>("Map");
        in_pose_ = modifier.addInput<TransformMessage>("Robot Pose");

        out_ss_ = modifier.addOutput<nav_msgs::OccupancyGrid>("Search Space");

        event_done_ = modifier.addEvent("done");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("velocity", 0.0, 3.0, 1.0, 0.01), velocity_);

        params.addParameter(param::ParameterFactory::declareBool("allow_backwards", false), allow_backwards);

        params.addParameter(param::ParameterFactory::declareRange("min_move_distance", 0.0, 10.0, 2.0, 0.01), min_move_distance);
        params.addParameter(param::ParameterFactory::declareRange("min_distance_to_obstacles", 0.0, 3.0, 0.75, 0.01), min_distance_to_obstacles);
        params.addParameter(param::ParameterFactory::declareRange("max_distance_to_unknown", 0.0, 3.0, 1.0, 0.01), max_distance_to_unknown);
    }

    void process()
    {
        const std::shared_ptr<nav_msgs::OccupancyGrid const>& map = msg::getMessage<nav_msgs::OccupancyGrid>(in_map_);
        const TransformMessage::ConstPtr pose = msg::getMessage<TransformMessage>(in_pose_);

        findExplorationPoint(*map, pose->value);
    }
private:
    Input* in_map_;
    Input* in_pose_;

    Output* out_ss_;

    Event* event_done_;

    double velocity_;
    bool allow_backwards;

    double min_move_distance;
    double min_distance_to_obstacles;
    double max_distance_to_unknown;

private:
    void findExplorationPoint(const nav_msgs::OccupancyGrid& map, const tf::Pose& own_pose)
    {
        tf::Vector3 own_pos = own_pose.getOrigin();
        cv::Point2i map_pos;
        map_pos.x = (own_pos.x() - map.info.origin.position.x) / map.info.resolution;
        map_pos.y = (own_pos.y() - map.info.origin.position.y) / map.info.resolution;

        splitMap(map, map_pos);

        // PUBLISH SEARCH SPACE AS GRID MAP
        auto ss = generateSearchSpace(map_pos, min_move_distance, min_distance_to_obstacles, max_distance_to_unknown);
        msg::publish(out_ss_, shared_ptr_tools::to_std_shared(ss));

        if(done_) {
            msg::trigger(event_done_);
        }
    }

    std::size_t index(const cv::Point2i& pt, std::size_t step) {
        return pt.y * step + pt.x;
    }
    std::size_t index(int x, int y, std::size_t step) {
        return y * step + x;
    }

    nav_msgs::OccupancyGridPtr generateSearchSpace(const cv::Point2i& map_pos,
                                                   double min_move_distance,
                                                   double min_distance_to_obstacles,
                                                   double max_distance_to_unknown)
    {
        int w = last_map.info.width;
        int h = last_map.info.height;

        nav_msgs::OccupancyGridPtr ss = boost::make_shared<nav_msgs::OccupancyGrid>();
        ss->header = last_map.header;
        ss->info = last_map.info;
        ss->data.resize(w*h, 0);

        double res = last_map.info.resolution;

        bool done = true;

        int8_t* dataP = &ss->data[0];
        for(int row = 0; row < h; ++row) {
            for(int col = 0; col < w; ++col) {
                float dist_to_obstacles = distance_to_obstacle.at<float>(row, col) * res;
                float dist_to_unknown = distance_to_unknown.at<float>(row, col) * res;
                float dist_to_robot = std::hypot(map_pos.x - col, map_pos.y - row) * res;

                if(dist_to_obstacles < min_distance_to_obstacles) {
                    *dataP = -1;
                } else {
                    if (dist_to_unknown < max_distance_to_unknown) {
                        if(dist_to_robot > min_move_distance) {
                            *dataP = 100;
                            done = false;
                        }
                    }
                }
                ++dataP;
            }
        }

        done_ = done;

        return ss;
    }

    void splitMap(const nav_msgs::OccupancyGrid &map, cv::Point2i map_pos)
    {
        last_map = map;

        int w = map.info.width;
        int h = map.info.height;

        const cv::Mat data(h, w, CV_8SC1, const_cast<signed char*>(map.data.data()));

        cv::Mat map_free(h, w, CV_8UC1, cv::Scalar::all(255));
        cv::Mat map_obstacle(h, w, CV_8UC1, cv::Scalar::all(255));
        cv::Mat map_unknown(h, w, CV_8UC1, cv::Scalar::all(255));

        for(int row = 0; row < h; ++row) {
            for(int col = 0; col < w; ++col) {
                const char& cell = data.at<char>(row, col);

                bool free = cell >= 0 && cell <= 50;
                bool occupied = cell > 50;

                double distance = std::hypot(row - map_pos.y, col - map_pos.x);
                if(distance < 10.0) {
                    free = true;
                    occupied = false;
                }

                if(free) {
                    map_free.at<uchar>(row, col) = 0;
                } else if(occupied) {
                    map_obstacle.at<uchar>(row, col) = 0;
                } else {
                    map_unknown.at<uchar>(row, col) = 0;
                }
            }
        }


        int kernel_size = 4;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*kernel_size + 1, 2*kernel_size+1 ),
                                                   cv::Point( kernel_size, kernel_size ) );
        int small_kernel_size = 3;
        cv::Mat small_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                         cv::Size( 2*small_kernel_size + 1, 2*small_kernel_size+1 ),
                                                         cv::Point( small_kernel_size, small_kernel_size ) );

        //    cv::dilate(map_free, map_free_safe, kernel);


        cv::dilate(map_free, map_free, kernel);
        cv::erode(map_obstacle, map_obstacle, kernel);
        cv::erode(map_unknown, map_unknown, small_kernel);

        cv::distanceTransform(map_obstacle, distance_to_obstacle, CV_DIST_L2, 5);
        assert(distance_to_obstacle.type() == CV_32FC1);

        cv::distanceTransform(map_unknown, distance_to_unknown, CV_DIST_L2, 5);
        assert(distance_to_unknown.type() == CV_32FC1);

        search_space = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(0));

        for(int row = 0; row < h; ++row) {
            for(int col = 0; col < w; ++col) {
                char& e = search_space.at<char>(row, col);

                //            bool free = 0 == map_free.at<char>(row, col);
                bool obstacle = 0 == map_obstacle.at<char>(row, col);
                bool unknown = 0 == map_unknown.at<char>(row, col);

                if(obstacle) {
                    e = OBSTACLE;
                } else if(unknown) {
                    e = UNKNOWN;
                } else {
                    const float& dist_to_obst = distance_to_obstacle.at<float>(row, col);
                    e = (254 - std::min(254.f, dist_to_obst * 10.0f));
                }
            }
        }
    }

private:
    enum STATES {
        OBSTACLE = 254,
        FREE = 0,
        UNKNOWN = 255
    };

private:
    nav_msgs::OccupancyGrid last_map;

    cv::Mat search_space;
    cv::Mat distance_to_obstacle;
    cv::Mat distance_to_unknown;

    bool done_;
};

}

CSAPEX_REGISTER_CLASS(csapex::GenerateExplorationMap, csapex::Node)

