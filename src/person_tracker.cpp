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
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/tf_listener.h>
#include <csapex/view/utility/color.hpp>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/generic_ros_message.h>
#include <csapex/model/token.h>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <person_msgs/Person.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class PersonTracker;

struct DynamicObject
{
    PersonTracker* tracker;

    int id;
    tf::Pose first_pose;
    tf::Pose pose;

    tf::Vector3 dim;

    std::deque<std::pair<ros::Time, tf::Pose>> history;

    //pcl::VoxelGrid<pcl::PointXYZ> pts;
    pcl::PointCloud<pcl::PointXYZ> pts;
    pcl::PointCloud<pcl::PointXYZ> latest_pts;

    tf::Vector3 vel;

    int life;
    int velocity_interval_;

    int count;
    int person_count;


    bool dynamic_;
    bool person_;

    DynamicObject(PersonTracker* tracker, int id, int velocity_interval, const tf::Pose& pose, const tf::Vector3& dim)
        : tracker(tracker), id(id), first_pose(pose), pose(pose), dim(dim), velocity_interval_(velocity_interval), count(0), person_count(0), dynamic_(false), person_(false)
    {
//        double res = 0.05;
        //pts.setLeafSize(res, res, res);
        vel = tf::Vector3(0,0,0);
    }

    void update(const ros::Time& time, const tf::Pose& measurement);

    double personProbability() const;
};

class PersonTracker : public Node
{
    friend struct DynamicObject;

public:
    PersonTracker()
        : has_current_transform_(false), next_id(0), tracking_id(-1)
    {}

    void setup(csapex::NodeModifier& modifier) override
    {
        last_process_ = ros::Time::now();

        modifier.addTypedSlot<GenericPointerMessage<person_msgs::Person>>("person detections", [this](const TokenPtr& token) {
            TokenDataConstPtr data = token->getTokenData();
            auto person = msg::message_cast<GenericPointerMessage<person_msgs::Person> const>(data);

            if(person) {
                process2dDetection(*person->value);
            } else {
                std::cerr << "cannot cast person message from " << type2name(typeid(*data)) << std::endl;
            }
        });

        in_tf_ = modifier.addInput<TransformMessage>("fixed frame -> base link");
        in_clusters_ = modifier.addInput<GenericVectorMessage, PointCloudMessage::ConstPtr>("clusters");

        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("Markers");
        out_people_ = modifier.addOutput<GenericVectorMessage, std::shared_ptr<person_msgs::Person>>("people");
        out_obstacles_ = modifier.addOutput<PointCloudMessage>("obstacles");
        out_map_ = modifier.addOutput<PointCloudMessage>("obstacle_map");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareInterval("z_range", -4.0, 4.0, 0.0, 2.0, 0.01), z_range_);
        params.addParameter(param::ParameterFactory::declareRange("max_distance", 0.0, 5.0, 1.5, 0.01), max_distance_);
        params.addParameter(param::ParameterFactory::declareRange("lifetime", 1, 100, 10, 1), lifetime_);
        params.addParameter(param::ParameterFactory::declareRange("look_ahead_duration", 0.0, 5.0, 1.5, 0.01), look_ahead_duration_);

        params.addParameter(param::ParameterFactory::declareRange("min_probability", 0.0, 1.0, 0.2, 0.001), min_probability_);
        params.addParameter(param::ParameterFactory::declareRange("min_count", 1, 64, 10, 1), [this](param::Parameter*p) {
            min_count_ = p->as<int>();
        });
        params.addParameter(param::ParameterFactory::declareRange("min_person_count", 1, 64, 3, 1), [this](param::Parameter*p) {
            min_person_count_ = p->as<int>();
        });

        params.addParameter(param::ParameterFactory::declareRange("max_dx",
                                                                  param::ParameterDescription("maximum deviation in the first principle componenent between two instances."),
                                                                  0.0, 1.0, 0.2, 0.001), max_dx_);
        params.addParameter(param::ParameterFactory::declareRange("max_dy",
                                                                  param::ParameterDescription("maximum deviation in the second principle componenent between two instances."),
                                                                  0.0, 1.0, 0.2, 0.001), max_dy_);
        params.addParameter(param::ParameterFactory::declareRange("max_dz",
                                                                  param::ParameterDescription("maximum deviation in height between two instances."),
                                                                  0.0, 2.0, 1.0, 0.001), max_dz_);

        params.addParameter(param::ParameterFactory::declareRange("velocity_interval", 1, 100, 10, 1), [this](param::Parameter* p){
            velocity_interval_ = p->as<int>();
            for(DynamicObject& o : objects_) {
                o.velocity_interval_ = velocity_interval_;
            }
        });

        params.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*){
            objects_.clear();
            next_id = 0;
            tracking_id = -1;
        });
    }

    void process()
    {
        //        TokenDataConstPtr data = token->getTokenData();
        //        auto vector = msg::message_cast<GenericVectorMessage const>(data);
        //        auto msg = vector->template makeShared<PointCloudMessage::ConstPtr>();

        auto m = msg::getMessage<GenericVectorMessage, PointCloudMessage::ConstPtr>(in_clusters_);

        std::vector<PointCloudMessage::ConstPtr> detections;
        for(const auto& token : *m) {
            detections.push_back(token);
        }

        if(raw_obstacles_) {
            raw_obstacles_->points.clear();
        }
        process3dDetections(detections);



        TransformMessage::ConstPtr tf_msg = msg::getMessage<TransformMessage>(in_tf_);
        current_transform_ = tf_msg->value;
        has_current_transform_ = true;

        tracking_frame_ = tf_msg->frame_id;

        ros::Time now = ros::Time::now();

        double dt = (now - last_process_).toSec();
        last_process_ = now;

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

        std::shared_ptr<std::vector<std::shared_ptr<person_msgs::Person const>>> person_vector =
                std::make_shared<std::vector<std::shared_ptr<person_msgs::Person const>>>();

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_map =
                boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        obstacles_map->header.frame_id = tracking_frame_;
        obstacles_map->header.stamp = tmp_time_.toNSec() * 1e-3;

        for(auto it = objects_.begin(); it != objects_.end(); ) {
            DynamicObject& obj = *it;
            --obj.life;
            if(obj.life <= 0) {
                if(tracking_id == obj.id) {
                    tracking_id = -1;
                }
                it = objects_.erase(it);
                continue;
            } else {
                ++it;
            }

            obj.pose.setOrigin(obj.pose.getOrigin() + obj.vel * dt);

            obj.vel *= 0.5;

            marker.action = visualization_msgs::Marker::ADD;

            if(isPerson(obj)) {
                tf::Pose p = obj.pose;
                p.setRotation(tf::createQuaternionFromYaw(std::atan2(obj.vel.y(), obj.vel.x())));

                geometry_msgs::Pose pose;
                tf::poseTFToMsg(p, pose);

                person_msgs::Person person;
                person.header.frame_id = tracking_frame_;
                person.header.stamp = now;
                person.pose = pose;
                person_vector->push_back(std::make_shared<person_msgs::Person>(person));


                marker.pose = pose;

                marker.type = visualization_msgs::Marker::ARROW;

                marker.scale.x = 0.1 + obj.vel.length();
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1;

            } else {
                if(obj.count > min_count_) {
                    if(obj.dynamic_) {
                        *obstacles_map += obj.latest_pts;

                    } else {
                        *obstacles_map += obj.pts;
                    }
                }

                geometry_msgs::Pose pose;
                tf::poseTFToMsg(obj.pose, pose);

                marker.pose = pose;

                marker.type = visualization_msgs::Marker::CYLINDER;

                marker.pose = pose;

                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.1;
                marker.color.a = 0.2;
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

        msg::publish<GenericVectorMessage, std::shared_ptr<person_msgs::Person const>>(out_people_, person_vector);


        PointCloudMessage::Ptr obstacles_map_msg = std::make_shared<PointCloudMessage>(obstacles_map->header.frame_id, obstacles_map->header.stamp);
        obstacles_map_msg->value = obstacles_map;
        msg::publish(out_map_, obstacles_map_msg);

        if(raw_obstacles_) {
            PointCloudMessage::Ptr obstacles = std::make_shared<PointCloudMessage>(tmp_frame_id_, tmp_time_.toNSec() * 1e-3);
            obstacles->value = raw_obstacles_;
            msg::publish(out_obstacles_, obstacles);
        }
    }

    bool isPerson(const DynamicObject& obj) const
    {
        return obj.person_;//obj.count >= velocity_interval_ && obj.personProbability() >= min_probability_;
    }

    void process2dDetection(const person_msgs::Person& person)
    {
        if(!has_current_transform_) {
            aerr << "transformation is not known" << std::endl;
            return;
        }

        tf::Transform pose;
        tf::poseMsgToTF(person.pose, pose);


        LockedTFListener l = TFListener::getLocked();
        apex_assert(l.l);
        auto listener = l.l->tfl;
        apex_assert(listener);
        tf::TransformListener& tfl_ = *listener;

        if(tfl_.waitForTransform(tracking_frame_, person.header.frame_id, tmp_time_, ros::Duration(0.1))) {
            tf::StampedTransform trafo;
            tfl_.lookupTransform(tracking_frame_, person.header.frame_id, tmp_time_, trafo);

            if(std::isnan(tf::getYaw(pose.getRotation()))) {
                pose.setRotation(tf::createIdentityQuaternion());
            }

            pose = trafo * pose;

            double min_dist = max_distance_;
            DynamicObject* best_fit = nullptr;
            for(DynamicObject& obj : objects_) {
                double z = obj.dim.z();
                if(z >= z_range_.first && z <= z_range_.second) {
                    double dist = (obj.pose.getOrigin() - pose.getOrigin()).length();
                    if(dist < min_dist) {
                        min_dist = dist;
                        best_fit = &obj;
                    }
                } else {
                    ainfo << "object of height " << z << " not considered a person" << std::endl;
                }
            }

            if(best_fit) {
                ainfo << "classify object at " << pose.getOrigin().x() << " / " << pose.getOrigin().y() << " as a person" << std::endl;
                best_fit->person_count++;

            } else {
                aerr << "cannot process detected person, no cluster is near enough" << std::endl;
            }
        } else {
            aerr << "cannot process detected person, transformation is unknown" << std::endl;
        }
    }

    void process3dDetections(const std::vector<PointCloudMessage::ConstPtr>& detections)
    {
        if(!has_current_transform_) {
            return;
        }

        if(detections.size() > 0) {
            tmp_frame_id_ = "";

            for(const PointCloudMessage::ConstPtr& pcl_msg : detections) {
                boost::apply_visitor(PointCloudMessage::Dispatch<PersonTracker>(this, pcl_msg), pcl_msg->value);
            }
        }
    }

    template<typename PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        if(tmp_frame_id_.size() == 0) {
            tmp_time_.fromNSec(cloud->header.stamp * 1e3);
            tmp_frame_id_ = cloud->header.frame_id;

            LockedTFListener l = TFListener::getLocked();
            apex_assert(l.l);
            auto listener = l.l->tfl;
            apex_assert(listener);
            tf::TransformListener& tfl_ = *listener;

            if(!tfl_.waitForTransform(tracking_frame_, tmp_frame_id_, tmp_time_, ros::Duration(0.05))) {
                return;
            }
            tfl_.lookupTransform(tracking_frame_, tmp_frame_id_, tmp_time_, tmp_trafo_);
        }

        Eigen::Vector4f center;
        Eigen::Matrix3f covariance;
        pcl::computeMeanAndCovarianceMatrix(*cloud, covariance, center);


        typename pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *xy_cloud);
        for(pcl::PointXYZ& pt : *xy_cloud) {
            pt.z = 0.0;
        }

        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(xy_cloud);

        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

        Eigen::Vector3f principal_component = eigen_vectors.block<3,1>(0,0);
        double theta = std::atan2(principal_component(1), principal_component(0));

        // rotate points
        for(pcl::PointXYZ& pt : *xy_cloud) {
            pcl::PointXYZ rotated;
            rotated.x = std::cos(-theta) * pt.x - std::sin(-theta) * pt.y;
            rotated.y = std::sin(-theta) * pt.x + std::cos(-theta) * pt.y;
            rotated.z = pt.z;
            pt = rotated;
        }

        // find aabb
        double x_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();

        for(const pcl::PointXYZ& p : *xy_cloud) {
            if(p.x < x_min) x_min = p.x;
            if(p.x > x_max) x_max = p.x;
            if(p.y < y_min) y_min = p.y;
            if(p.y > y_max) y_max = p.y;
        }

        double z_min = std::numeric_limits<double>::infinity();
        double z_max = -std::numeric_limits<double>::infinity();

        const PointT* pt = &cloud->front();
        for(std::size_t i = 0, n = cloud->points.size(); i < n; ++i, ++pt) {
            const PointT& p = *pt;
            if(p.z < z_min) z_min = p.z;
            if(p.z > z_max) z_max = p.z;
        }

        double length = (x_max - x_min);
        double width = (y_max - y_min);

        if(width > length) {
            std::swap(width, length);
        }

        double height = (z_max - z_min);

        tf::Vector3 dim(length, width, height);


        geometry_msgs::PoseWithCovarianceStamped measurement;
        measurement.header.frame_id = tmp_frame_id_;
        measurement.header.stamp = tmp_time_;
        measurement.pose.pose.position.x = center[0];
        measurement.pose.pose.position.y = center[1];
        measurement.pose.pose.position.z = center[2];
        measurement.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
        measurement.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
        measurement.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
        measurement.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                measurement.pose.covariance[i * 6 + j] = covariance(i, j);

        tf::Transform pose;
        tf::poseMsgToTF(measurement.pose.pose, pose);

        if(std::isnan(tf::getYaw(pose.getRotation()))) {
            pose.setRotation(tf::createIdentityQuaternion());
        }

        pose = tmp_trafo_ * pose;

        double min_dist = max_distance_;
        DynamicObject* best_fit = nullptr;
        for(DynamicObject& obj : objects_) {
            double dist = (obj.pose.getOrigin() - pose.getOrigin()).length();
            if(dist < min_dist) {
                double dx = std::abs(obj.dim.x() - dim.x());
                if(dx < max_dx_) {
                    double dy = std::abs(obj.dim.y() - dim.y());
                    if(dy < max_dy_) {
                        double dz = std::abs(obj.dim.z() - dim.z());
                        if(dz < max_dz_) {
                            min_dist = dist;
                            best_fit = &obj;
                        }
                    }
                }
            }
        }

        if(!raw_obstacles_) {
            raw_obstacles_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            raw_obstacles_->header = cloud->header;
        }

        raw_obstacles_->header.stamp = cloud->header.stamp;

        if(!best_fit || !isPerson(*best_fit)) {
            for(const PointT& p : *cloud) {
                pcl::PointXYZ pt;
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                raw_obstacles_->points.push_back(pt);
            }
        }

        if(!best_fit) {
            ainfo << "new object at " << pose.getOrigin().x() << " / " << pose.getOrigin().y() << std::endl;
            objects_.emplace_back(this, next_id++, velocity_interval_, pose, dim);
            best_fit = &objects_.back();
        } else {
            best_fit->dim = dim;
        }

        best_fit->latest_pts.clear();

        for(const PointT& pt : *cloud) {
            tf::Vector3 point = tmp_trafo_ * tf::Vector3 (pt.x, pt.y, pt.z);
            pcl::PointXYZ pcl_pt;
            pcl_pt.x = point.x();
            pcl_pt.y = point.y();
            pcl_pt.z = point.z();
            best_fit->pts.push_back(pcl_pt);

            best_fit->latest_pts.push_back(pcl_pt);
        }

        best_fit->update(measurement.header.stamp, pose);
        best_fit->life = lifetime_;
    }

private:
    Input* in_clusters_;
    Input* in_tf_;

    Output* out_marker_;
    Output* out_people_;
    Output* out_obstacles_;
    Output* out_map_;

    ros::Time last_process_;

    bool has_current_transform_;
    std::string tracking_frame_;
    tf::Transform current_transform_;

    std::vector<DynamicObject> objects_;

    int next_id;
    int tracking_id;

    int velocity_interval_;
    double look_ahead_duration_;

    double min_probability_;
    double min_count_;
    double min_person_count_;

    double max_dx_;
    double max_dy_;
    double max_dz_;

    double max_distance_;
    std::pair<double, double> z_range_;
    int lifetime_;

    tf::StampedTransform tmp_trafo_;

    geometry_msgs::PoseWithCovarianceStamped tmp_pose_;
    std::string tmp_frame_id_;
    ros::Time tmp_time_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_obstacles_;
};

void DynamicObject::update(const ros::Time& time, const tf::Pose& measurement)
{
    pose = measurement;
    history.push_back(std::make_pair(time, measurement));

    while(history.size() > (std::size_t) velocity_interval_) {
        history.pop_front();
    }

    if(history.size() == (std::size_t) velocity_interval_) {
        tf::Vector3 delta = pose.getOrigin() - history.front().second.getOrigin();

        if(!dynamic_) {
            tf::Vector3 delta_start = pose.getOrigin() - first_pose.getOrigin();
            if(delta_start.length() > 1.0) {
                dynamic_ = true;
            }
        }

        ros::Duration dt = time - history.front().first;

        tf::Vector3 new_vel = (1.0 / dt.toSec()) * delta;
        if(new_vel.length() < 0.25) {
            // keep orientation for small velocities
            vel = vel / vel.length() * new_vel.length();
        } else {
            vel = new_vel;
        }
    }

    ++count;

    if(!person_ && dynamic_) {
        if(personProbability() > tracker->min_probability_) {
            person_ = true;
        }
    }
}

double DynamicObject::personProbability() const
{
    if(count < tracker->min_count_ || person_count < tracker->min_person_count_) {
        return 0.0;
    }
    return person_count / (double) count;
}

}

CSAPEX_REGISTER_CLASS(csapex::PersonTracker, csapex::Node)

