/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_transform/transform_message.h>
#include <csapex/model/token.h>
#include <csapex/signal/event.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/tf_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <csapex/view/utility/color.hpp>


using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

struct Object
{
    int id;
    tf::Pose pose;

    std::deque<std::pair<ros::Time, tf::Pose>> history;

    tf::Vector3 vel;

    int life;
    int velocity_interval_;

    Object(int id, int velocity_interval, const tf::Pose& pose)
        : id(id), pose(pose), velocity_interval_(velocity_interval)
    {
        vel = tf::Vector3(0,0,0);
    }

    void update(const ros::Time& time, const tf::Pose& measurement)
    {
        pose = measurement;
        history.push_back(std::make_pair(time, measurement));

        while(history.size() > velocity_interval_) {
            history.pop_front();
        }
        if(history.size() == velocity_interval_) {
            tf::Vector3 delta = pose.getOrigin() - history.front().second.getOrigin();
            ros::Duration dt = time - history.front().first;

            vel = (1.0 / dt.toSec()) * delta;
        }
    }
};

class ObjectTracker : public Node
{
public:
    ObjectTracker()
        : has_current_transform_(false), next_id(0), tracking_id(-1)
    {}

    void setup(csapex::NodeModifier& modifier) override
    {
        last_process_ = ros::Time::now();

        modifier.addTypedSlot<GenericVectorMessage, std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>>("3D detections", [this](const TokenPtr& token) {
            TokenDataConstPtr data = token->getTokenData();
            auto vector = msg::message_cast<GenericVectorMessage const>(data);
            auto msg = vector->template makeShared<std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>>();

            std::vector<geometry_msgs::PoseWithCovarianceStamped> detections;
            for(const std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>& token : *msg) {
                detections.push_back(*token);
            }

            process3dDetections(detections);
        });

        in_tf_ = modifier.addInput<TransformMessage>("fixed frame -> base link");

        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("Markers");
        out_target_ = modifier.addOutput<TransformMessage>("target");

        event_start_tracking_ = modifier.addEvent("start tracking");
        event_stop_tracking_ = modifier.addEvent("stop tracking");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareInterval("z_range", -4.0, 4.0, 0.0, 2.0, 0.01), z_range_);
        params.addParameter(param::ParameterFactory::declareRange("max_distance", 0.0, 5.0, 1.5, 0.01), max_distance_);
        params.addParameter(param::ParameterFactory::declareRange("lifetime", 1, 100, 10, 1), lifetime_);
        params.addParameter(param::ParameterFactory::declareRange("look_ahead_duration", 0.0, 5.0, 1.5, 0.01), look_ahead_duration_);



        params.addParameter(param::ParameterFactory::declareRange("velocity_interval", 1, 100, 10, 1), [this](param::Parameter* p){
            velocity_interval_ = p->as<int>();
            for(Object& o : objects_) {
                o.velocity_interval_ = velocity_interval_;
            }
        });

        params.addParameter(param::ParameterFactory::declareInterval("tracking_start_x_range", -4.0, 4.0, 0.0, 2.0, 0.01), start_x_range_);
        params.addParameter(param::ParameterFactory::declareInterval("tracking_start_y_range", -4.0, 4.0, -0.5, 0.5, 0.01), start_y_range_);
        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*){
            objects_.clear();
            next_id = 0;
            tracking_id = -1;
        });
    }

    void process()
    {
        TransformMessage::ConstPtr tf_msg = msg::getMessage<TransformMessage>(in_tf_);
        current_transform_ = tf_msg->value;
        has_current_transform_ = true;

        tracking_frame_ = tf_msg->frame_id;

        ros::Time now = ros::Time::now();

        double dt = (now - last_process_).toSec();
        last_process_ = now;

        TransformMessage::ConstPtr trafo_msg = msg::getMessage<TransformMessage>(in_tf_);
        tf::Transform world_to_base_link = trafo_msg->value;

        auto message = std::make_shared<visualization_msgs::MarkerArray>();
        visualization_msgs::MarkerArray& marker_array = *message;

        visualization_msgs::Marker marker;
        marker.header.frame_id = tracking_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "people";
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker.color.a = 0.7;
        marker.id = 0;

        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);

        TransformMessage::Ptr target(new TransformMessage(tracking_frame_, "target"));

        for(auto it = objects_.begin(); it != objects_.end(); ) {
            Object& obj = *it;
            --obj.life;
            if(obj.life <= 0) {
                if(tracking_id == obj.id) {
                    tracking_id = -1;
                    event_stop_tracking_->trigger();
                }
                it = objects_.erase(it);
                continue;
            } else {
                ++it;
            }

            if(tracking_id == obj.id) {
                target->value = obj.pose;

                target->value.setOrigin(target->value.getOrigin() + obj.vel * look_ahead_duration_);
            }

            obj.pose.setOrigin(obj.pose.getOrigin() + obj.vel * dt);

            obj.vel *= 0.5;

            marker.action = visualization_msgs::Marker::ADD;

            if(obj.vel.length() < 1e-8) {
                geometry_msgs::Pose pose;
                tf::poseTFToMsg(obj.pose, pose);

                marker.pose = pose;

                marker.type = visualization_msgs::Marker::CYLINDER;

                marker.pose = pose;

                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.1;
                marker.color.a = 0.2;

            } else {
                tf::Pose p = obj.pose;
                p.setRotation(tf::createQuaternionFromYaw(std::atan2(obj.vel.y(), obj.vel.x())));

                geometry_msgs::Pose pose;
                tf::poseTFToMsg(p, pose);

                marker.pose = pose;

                marker.type = visualization_msgs::Marker::ARROW;

                marker.scale.x = 0.1 + obj.vel.length();
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1;
            }


            double r = 0,g = 0,b = 0;
            color::fromCount(obj.id, r,g,b);
            marker.color.r = r / 255.;
            marker.color.g = g / 255.;
            marker.color.b = b / 255.;

            ++marker.id;

            marker_array.markers.push_back(marker);
        }

        msg::publish(out_marker_, message);

        if(tracking_id != -1) {
            msg::publish(out_target_, target);
        } else {

            tf::Transform base_link_to_world = world_to_base_link.inverse();

            ainfo << "searching for candidate: "<< std::endl;
            std::vector<const Object*> candidates;
            for(const Object& o : objects_) {
                tf::Pose local_pose = base_link_to_world * o.pose;

                double x = local_pose.getOrigin().x();
                double y = local_pose.getOrigin().y();

                ainfo << "-  " << x << " / " << y << std::endl;

                if(x > start_x_range_.first && x < start_x_range_.second &&
                        y > start_y_range_.first && y < start_y_range_.second) {
                    candidates.push_back(&o);
                }
            }

            if(candidates.size() == 1) {
                tracking_id = candidates.front()->id;
                event_start_tracking_->trigger();
            } else if(!candidates.empty()) {
                awarn << "too many candidates" << std::endl;
            }
        }
    }

    void correction(const tf::Transform& tf, const std::vector<geometry_msgs::PoseWithCovarianceStamped>& measurements)
    {
        for(const geometry_msgs::PoseWithCovarianceStamped& measurement : measurements) {
            tf::Transform pose;
            tf::poseMsgToTF(measurement.pose.pose, pose);

            double z = pose.getOrigin().z();
            if(z < z_range_.first || z > z_range_.second) {
                continue;
            }

            if(std::isnan(tf::getYaw(pose.getRotation()))) {
                pose.setRotation(tf::createIdentityQuaternion());
            }

            pose = tf * pose;

            double min_dist = max_distance_;
            Object* best_fit = nullptr;
            for(Object& obj : objects_) {
                double dist = (obj.pose.getOrigin() - pose.getOrigin()).length();
                if(dist < min_dist) {
                    min_dist = dist;
                    best_fit = &obj;
                }
            }

            if(!best_fit) {
                ainfo << "new object at " << pose.getOrigin().x() << " / " << pose.getOrigin().y() << std::endl;
                objects_.emplace_back(next_id++, velocity_interval_, pose);
                best_fit = &objects_.back();
            }

            best_fit->update(measurement.header.stamp, pose);
            best_fit->life = lifetime_;
        }
    }

    void process3dDetections(const std::vector<geometry_msgs::PoseWithCovarianceStamped>& detections)
    {
        if(has_current_transform_ && detections.size() > 0) {
            const geometry_msgs::PoseWithCovarianceStamped& p0 = detections.at(0);
            std::string frame_id = p0.header.frame_id;
            ros::Time t = p0.header.stamp;

            LockedTFListener l = TFListener::getLocked();
            if(!l.l) {
                node_modifier_->setError("cannot get transform listener");
                return;
            }
            tf::TransformListener& tfl_ = *l.l->tfl;

            if(tfl_.waitForTransform(tracking_frame_, frame_id, t, ros::Duration(0.05))) {
                tf::StampedTransform trafo;
                tfl_.lookupTransform(tracking_frame_, frame_id, t, trafo);
                correction(trafo, detections);
            }
        }
    }

private:
    Input* in_tf_;

    Output* out_marker_;
    Output* out_target_;

    Event* event_start_tracking_;
    Event* event_stop_tracking_;

    ros::Time last_process_;

    bool has_current_transform_;
    std::string tracking_frame_;
    tf::Transform current_transform_;

    std::vector<Object> objects_;

    int next_id;
    int tracking_id;

    int velocity_interval_;
    double look_ahead_duration_;

    double max_distance_;
    std::pair<double, double> z_range_;
    std::pair<double, double> start_x_range_;
    std::pair<double, double> start_y_range_;
    int lifetime_;
};

}

CSAPEX_REGISTER_CLASS(csapex::ObjectTracker, csapex::Node)

