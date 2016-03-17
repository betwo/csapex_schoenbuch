/// HEADER
#include "pillar_extractor.h"

/// SYSTEM
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

struct PillarExtractorImpl {
    PillarExtractorImpl(PillarExtractor* parent)
        : parent_(parent)
    {
    }

    ~PillarExtractorImpl() {

    }

    std::vector<Pillar> findPillars(const sensor_msgs::PointCloud2::ConstPtr& input);

    void filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& full_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          std::vector<pcl::PointIndices>& cluster_indices);

    void findPillars(std::vector<pcl::PointIndices>& cluster_indices,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                     std::vector<Pillar>& pillar_candidates);

    PillarExtractor* parent_;
};


PillarExtractor::PillarExtractor()
    : pimpl(new PillarExtractorImpl(this))
{

}

PillarExtractor::~PillarExtractor()
{
    delete pimpl;
}

std::vector<Pillar> PillarExtractor::findPillars(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    return pimpl->findPillars(input);
}

std::vector<Pillar> PillarExtractorImpl::findPillars(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*full_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<pcl::PointIndices> cluster_indices;

    filterPointCloud(full_cloud, cloud, cluster_indices);

    std::vector<Pillar> pillar_candidates;

    findPillars(cluster_indices, cloud, pillar_candidates);

    return pillar_candidates;
}

void PillarExtractorImpl::filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& full_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      std::vector<pcl::PointIndices>& cluster_indices)
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setFilterFieldName("intensity");
    pt.setFilterLimits(parent_->min_intensity_, 4096);
    pt.setInputCloud(full_cloud);
    pt.filter(*cloud);

    double d = parent_->max_range_;
    Eigen::Vector4f min_pt_(-d, -d, -d,  0);
    Eigen::Vector4f max_pt_(d, d, d, 0);

    pcl::CropBox<pcl::PointXYZI> box;
    box.setMin(min_pt_);
    box.setMax(max_pt_);
    box.setInputCloud(cloud);
    box.filter(*cloud);

    ROS_INFO_STREAM("high intensity points: " << cloud->size());

    if(cloud->empty()) {
        return;
    }

    typename pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);

    typename pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (parent_->cluster_tolerance_);
    ec.setMinClusterSize (parent_->min_cluster_size_);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    ROS_INFO_STREAM(cluster_indices.size() << " cluster points");
}


void PillarExtractorImpl::findPillars(std::vector<pcl::PointIndices>& cluster_indices,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 std::vector<Pillar>& pillar_candidates)
{
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
        it != cluster_indices.end();
        ++it)
    {
        const pcl::PointIndices& indices_msg = *it;

        if((int) indices_msg.indices.size() < parent_->min_pts_) {
            continue;
        }

        double angle = 0;
        double dist_sqr = std::numeric_limits<double>::infinity();

        for(std::vector<int>::const_iterator it = indices_msg.indices.begin();
            it != indices_msg.indices.end();
            ++it)
        {
            const pcl::PointXYZI& pt = cloud->points[*it];

            double d_sqr = (pt.x * pt.x + pt.y * pt.y/* + pt.z * pt.z*/);
            if(d_sqr < dist_sqr) {
                dist_sqr = d_sqr;
                angle = std::atan2(pt.y, pt.x);
            }

        }

        if(std::isfinite(dist_sqr)) {
            double r = std::sqrt(dist_sqr) + parent_->radius_;

            Eigen::Vector3d centre(std::cos(angle) * r,
                               std::sin(angle) * r,
                               0.0);

            pillar_candidates.push_back(Pillar(centre, indices_msg.indices.size()));
        }
    }

    std::sort(pillar_candidates.begin(), pillar_candidates.end(), std::greater<Pillar>());
}
