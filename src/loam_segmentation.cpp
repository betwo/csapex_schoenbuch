/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_transform/transform_message.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex_ros/tf_listener.h>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <pcl/filters/voxel_grid.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}

class LoamSegmentation : public Node
{
public:
    LoamSegmentation()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");

        out_ = modifier.addOutput<PointCloudMessage>("Segment");
        out_curv_ = modifier.addOutput<PointCloudMessage>("Curvature");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        typename pcl::PointCloud<pcl::PointXYZI>::Ptr input = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(msg->value);
        if(input) {
            const pcl::PointCloud<pcl::PointXYZI>& cloud = *input;
            pcl::PointCloud<pcl::PointXYZI>::Ptr res(new pcl::PointCloud<pcl::PointXYZI>);
            res->header = input->header;

            segment(cloud, *res);

            PointCloudMessage::Ptr output_msg(new PointCloudMessage(cloud.header.frame_id, cloud.header.stamp));
            output_msg->value = res;

            msg::publish(out_, output_msg);
        }
    }

    void segment(const pcl::PointCloud<pcl::PointXYZI>& in, pcl::PointCloud<pcl::PointXYZI>& out)
    {
        int cloudSize = in.points.size();

        float startOri = -atan2(in.points[0].y, in.points[0].x);
        float endOri = -atan2(in.points[cloudSize - 1].y,
                in.points[cloudSize - 1].x) + 2 * M_PI;

        if (endOri - startOri > 3 * M_PI) {
            endOri -= 2 * M_PI;
        } else if (endOri - startOri < M_PI) {
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;
        pcl::PointXYZI point;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr curvCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudScans[17];

        float cloudCurvature[40000];
        int cloudSortInd[40000];
        int cloudNeighborPicked[40000];
        int cloudLabel[40000];

        int scanStartInd[17];
        int scanEndInd[17];


        for (int i = 0; i < 17; i++) {
            laserCloudScans[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

        int nNan = 0;
        for (int i = 0; i < cloudSize; i++) {
            point.x = in.points[i].x;
            point.y = in.points[i].y;
            point.z = in.points[i].z;


            float angle = atan(point.z / sqrt(point.y * point.y + point.x * point.x)) * 180. / M_PI;
            int scanID = int(angle*0.5 + 0.5) + 8;
            //     std::cout << point << std::endl;
            //     std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;
            //     std::cout << "angle: " << angle << " scanID: " << scanID << std::endl;
            if (std::isnan(angle)) {
                nNan++;
                continue;
            }

            if (angle < 0) {
                scanID--;
            }

            if(scanID < 0) {
                continue;
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed) {
                if (ori < startOri - M_PI / 2) {
                    ori += 2 * M_PI;
                } else if (ori > startOri + M_PI * 3 / 2) {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI) {
                    halfPassed = true;
                }
            } else {
                ori += 2 * M_PI;

                if (ori < endOri - M_PI * 3 / 2) {
                    ori += 2 * M_PI;
                } else if (ori > endOri + M_PI / 2) {
                    ori -= 2 * M_PI;
                }
            }

            float relTime = (ori - startOri) / (endOri - startOri);
            point.intensity = scanID + 0.1 * relTime;

            laserCloudScans[scanID]->push_back(point);
        }


        for (int i = 0; i < 17; i++) {
            *laserCloud += *laserCloudScans[i];
        }

        cloudSize = laserCloud->points.size();

        curvCloud->header = laserCloud->header;
        curvCloud->points.resize(cloudSize);
        for(int i = 0; i < cloudSize; ++i) {
            const auto& in = laserCloud->points[i];
            auto& out = curvCloud->points[i];

            out.x = in.x;
            out.y = in.y;
            out.z = in.z;
        }

        int scanCount = -1;
        for (int i = 5; i < cloudSize - 5; i++) {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                    + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                    + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                    + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                    + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                    + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                    + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                    + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                    + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                    + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                    + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                    + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                    + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                    + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                    + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                    + laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;


            auto& out = curvCloud->points[i];
            out.r = cloudCurvature[i];
            out.g = 0;
            out.b = 0;

            if(cloudCurvature[i] > 5) {
                out.x = out.y = out.z = std::numeric_limits<double>::quiet_NaN();
            }

            if (int(laserCloud->points[i].intensity) != scanCount) {
                scanCount = int(laserCloud->points[i].intensity);

                if (scanCount > 0) {
                    scanStartInd[scanCount] = i + 5;
                    scanEndInd[scanCount - 1] = i - 5;
                }
            }
        }
        scanStartInd[0] = 5;
        scanEndInd[16] = cloudSize - 5;

        for (int i = 5; i < cloudSize - 6; i++) {
            float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
            float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
            float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

            if (diff > 0.1) {

                float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                                    laserCloud->points[i].y * laserCloud->points[i].y +
                                    laserCloud->points[i].z * laserCloud->points[i].z);

                float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                        laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                        laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

                if (depth1 > depth2) {
                    diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
                    diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
                    diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
                        cloudNeighborPicked[i - 5] = 1;
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    }
                } else {
                    diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                    diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                    diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }
            }

            float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
            float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
            float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
            float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

            float dis = laserCloud->points[i].x * laserCloud->points[i].x
                    + laserCloud->points[i].y * laserCloud->points[i].y
                    + laserCloud->points[i].z * laserCloud->points[i].z;

            if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
                cloudNeighborPicked[i] = 1;
            }
        }

        for (int i = 0; i < 17; i++) {
            surfPointsLessFlatScan->clear();
            for (int j = 0; j < 6; j++) {
                int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
                int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;

                for (int k = sp + 1; k <= ep; k++) {
                    for (int l = k; l >= sp + 1; l--) {
                        if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
                            int temp = cloudSortInd[l - 1];
                            cloudSortInd[l - 1] = cloudSortInd[l];
                            cloudSortInd[l] = temp;
                        }
                    }
                }

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSortInd[k];
                    if (cloudNeighborPicked[ind] == 0 &&
                            cloudCurvature[ind] > 0.1) {

                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x
                                    - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y
                                    - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z
                                    - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            float diffX = laserCloud->points[ind + l].x
                                    - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y
                                    - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z
                                    - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSortInd[k];
                    if (cloudNeighborPicked[ind] == 0 &&
                            cloudCurvature[ind] < 0.1) {

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x
                                    - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y
                                    - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z
                                    - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            if(ind + l >= 0) {
                                float diffX = laserCloud->points[ind + l].x
                                        - laserCloud->points[ind + l + 1].x;
                                float diffY = laserCloud->points[ind + l].y
                                        - laserCloud->points[ind + l + 1].y;
                                float diffZ = laserCloud->points[ind + l].z
                                        - laserCloud->points[ind + l + 1].z;
                                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                    break;
                                }

                                cloudNeighborPicked[ind + l] = 1;
                            }
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
            }

            surfPointsLessFlatScanDS->clear();
            pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            *surfPointsLessFlat += *surfPointsLessFlatScanDS;
        }

        out = *surfPointsFlat;



        PointCloudMessage::Ptr output_msg(new PointCloudMessage(in.header.frame_id, in.header.stamp));
        output_msg->value = curvCloud;

        msg::publish(out_curv_, output_msg);
    }

private:
    Input* in_;
    Output* out_;
    Output* out_curv_;
};


}

CSAPEX_REGISTER_CLASS(csapex::LoamSegmentation, csapex::Node)

