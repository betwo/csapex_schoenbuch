/// HEADER
#include <pillar_localization/pillar_extractor.h>

/// SYSTEM
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


PillarExtractor::PillarExtractor()
    : cluster_distance_euclidean_(0.0)
{

}

PillarExtractor::~PillarExtractor()
{
}

std::vector<Pillar> PillarExtractor::findPillars(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& c)
{
    const pcl::PointCloud<pcl::PointXYZI>& cloud = *c;
    int cols = cloud.width;
    int rows = cloud.height;

    points.clear();
    clusters.clear();
    row_clusters.clear();

    points.resize(cloud.size());

    {
        for(int col = 0; col < cols; ++col) {
            for(int row = 0; row < rows; ++row) {
                const pcl::PointXYZI& pt = cloud.at(col, row);
                Point& pt_out = points.at(row * cols + col);

                pt_out.x = pt.x;
                pt_out.y = pt.y;
                pt_out.z = pt.z;
                pt_out.intensity = pt.intensity;

                pt_out.row = row;
                pt_out.col = col;
            }
        }
    }


    {
        // generate segments
        clusters.reserve(points.size());
        row_clusters.resize(rows);

        for(int row = 0; row < rows; ++row) {
            Point* last = NULL;
            clusters.push_back(Cluster());
            Cluster* current_cluster = &clusters.back();
            row_clusters[row].reserve(cols);
            row_clusters[row].push_back(current_cluster);

            for(int col = 0; col < cols; ++col) {
                Point& current = points[row * cols + col];
                if(std::isnan(current.x)) continue;

                if(last) {
                    double jump_distance = std::abs(last->range() - current.range());
                    //                        double jump_distance = last->distanceXYZ(current);

                    current.jump_distance = jump_distance;

                    if(jump_distance > cluster_distance_ring_) {
                        // new cluster
                        if((int) current_cluster->pts.size() >= cluster_min_size_) {
                            clusters.push_back(Cluster());
                            current_cluster = &clusters.back();
                            row_clusters[row].push_back(current_cluster);
                        } else {
                            current_cluster->clear();
                        }
                        current_cluster->col_start = col;
                    }
                }

                current_cluster->add(&current);
                current_cluster->col_end = col;

                last = &current;
            }

            for(std::size_t i = 0, n = row_clusters[row].size(); i < n; ++i) {
                Cluster* c  = row_clusters[row][i];
                if((int) c->pts.size() < cluster_min_size_ || (int) c->pts.size() > cluster_max_size_) {
                    c->clear();

                } else if(cluster_max_diameter_ > 0.0 && !c->empty()) {
                    double diameter = c->pts.front()->distanceXYZ(*c->pts.back());
                    if(diameter > cluster_max_diameter_) {
                        c->clear();
                    }
                }
            }
        }
    }

    {
        // cluster row based
        bool change = true;
        while(change) {
            change = false;

            for(int row = 0; row < rows; ++row) {
                for(int row2 = row + 1; row2 < rows; ++row2) {
                    if(row == row2) continue;

                    for(std::size_t i = 0, n = row_clusters[row].size(); i < n; ++i) {
                        Cluster* c1 = row_clusters[row][i];
                        if(c1->empty()) continue;

                        for(std::size_t j = 0, n = row_clusters[row2].size(); j < n; ++j) {
                            Cluster* c2 = row_clusters[row2][j];
                            if(c1 != c2) {
                                if(c2->empty()) continue;
                                if(c2->col_start > c1->col_end || c1->col_start > c2->col_end) continue;

                                for(std::size_t k = 0, n = c1->pts.size(); k < n; ++k) {
                                    Point* p1 = c1->pts[k];
                                    bool merged = false;
                                    for(std::size_t l = 0, m = c2->pts.size(); l < m; ++l) {
                                        Point* p2 = c2->pts[l];
                                        double distance = p1->distanceXY(*p2);
                                        if(distance < cluster_distance_vertical_) {
                                            c1->merge(c2);
                                            merged = true;
                                            change = true;
                                            break;
                                        }
                                    }
                                    if(merged) break;
                                }
                            }
                        }
                    }

                    for(std::vector<Cluster*>::iterator it = row_clusters[row].begin(); it != row_clusters[row].end();) {
                        if((*it)->empty()) {
                            it = row_clusters[row].erase(it);
                        } else {
                            ++it;
                        }
                    }
                }
            }
        }
    }

    // filter intensity
    std::vector<Cluster*> filtered_clusters;
    for(std::size_t i = 0, n = clusters.size(); i < n; ++i) {
        Cluster& c = clusters[i];
        if((int) c.pts.size() < pillar_min_points_) {
            c.clear();
        } else {
            std::vector<float> intensities(c.pts.size(), 0.0f);
            for(std::size_t j = 0, n = c.pts.size(); j < n; ++j) {
                Point* p = c.pts[j];
                intensities[j++] = p->intensity;
            }

            std::sort(intensities.begin(), intensities.end());

            if(intensities.empty() || intensities.back() < pillar_min_intensity_) {
                c.clear();
            } else {
                filtered_clusters.push_back(&c);
            }
        }
    }


    {
        // cluster euclidean
        for(std::vector<Cluster*>::iterator it = filtered_clusters.begin(); it != filtered_clusters.end();++it) {
            Cluster* c1 = *it;
            if(!c1->empty()) {
                for(std::vector<Cluster*>::iterator it2 = it + 1; it2 != filtered_clusters.end();++it2) {
                    Cluster* c2 = *it2;
                    if(c1 != c2 && !c2->empty()) {
                        for(std::size_t k = 0, n = c1->pts.size(); k < n; ++k) {
                            Point* p1 = c1->pts[k];
                            bool merged = false;
                            for(std::size_t l = 0, n = c2->pts.size(); l < n; ++l) {
                                Point* p2 = c2->pts[l];
                                double distance = p1->distanceXYZ(*p2);
                                if(distance < cluster_distance_euclidean_) {
                                    c1->merge(c2);
//                                    ROS_INFO_STREAM("euclidean merging");
                                    merged = true;
                                    break;
                                }
                            }
                            if(merged) break;
                        }
                    }
                }
            }

        }
    }


    // normalize cluster ids
    int id = 0;
    for(std::size_t i = 0, n = clusters.size(); i < n; ++i) {
        Cluster& c = clusters[i];
        if(c.pts.size() > 0) {
            c.id = id++;
        } else {
            c.id = -1;
        }
    }

    std::vector<Pillar> pillars;

    for(std::size_t i = 0, n = clusters.size(); i < n; ++i) {
        Cluster& c = clusters[i];
        if(c.empty()) {
            continue;
        }

        double r;
        Eigen::Vector3d C, W;
        double error = fitCylinder(c.pts, r, C, W);


        if(error < 1e-4 &&
                r > pillar_radius_ - pillar_radius_fuzzy_ &&
                r < pillar_radius_ + pillar_radius_fuzzy_) {
            Pillar p;
            p.centre = C;
            p.up = W;
            p.measured_radius = r;
            pillars.push_back(p);

//            ROS_INFO_STREAM("found pillar with " << c.pts.size() << " points" <<
//                            ", error: " << error << ". r: " << r <<
//                            ", up: " << W << ", centre: " << C);
        } else {

//            ROS_WARN_STREAM("reject pillar with " << c.pts.size() << " points" <<
//                            ", error: " << error << ". r: " << r <<
//                            ", up: " << W << ", centre: " << C);
        }
    }

    return pillars;
}


double PillarExtractor::fitCylinder(const std::vector<Point*>& cluster,
                                    double& r, Eigen::Vector3d& C, Eigen::Vector3d& W)
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > X;
    std::size_t n = cluster.size();

    X.reserve(n);

    Eigen::Vector3d average = Eigen::Vector3d::Zero();
    for(std::size_t i = 0; i < n; ++i) {
        Point* pt = cluster[i];
        Eigen::Vector3d p;
        p << pt->x, pt->y, pt->z;

        X.push_back(p);

        average += p;
    }
    average /= (double) n;
    for(std::size_t i = 0; i < n; ++i) {
        Eigen::Vector3d& pt = X[i];
        pt -= average;
    }

    double min_error = std::numeric_limits<double>::infinity();
    double rsqr = 0.0;

    int i_max = 64;
    int j_max = 64;

    for(int j = 0; j < j_max; ++j) {
        double phi = M_PI_2 * j / (double) j_max;
        double cos_phi = std::cos(phi);
        double sin_phi = std::sin(phi);

        for(int i = 0; i < i_max; ++i) {
            double theta = 2 * M_PI * i / (double) i_max;
            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);

            Eigen::Vector3d currentW(cos_theta * sin_phi, sin_theta * sin_phi, cos_phi);

            Eigen::Vector3d currentC;
            double current_rsqr;

            double error = G(n, X, currentW, currentC, current_rsqr);

            if(error < min_error) {
                min_error = error;
                W = currentW;
                C = currentC;
                rsqr = current_rsqr;
            }
        }
    }

    C += average;

    r = std::sqrt(rsqr);

    return min_error;
}

double PillarExtractor::G(std::size_t n,
                          const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& X,
                          const Eigen::Vector3d& W,
                          Eigen::Vector3d& PC,
                          double& rsqr)
{
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - W * W.transpose();
    Eigen::Matrix3d S;
    S << 0, -W(2), W(1),
            W(2), 0, -W(0),
            -W(1), W(0), 0;
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d B = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Y(n);
    double averageSqrLength = 0;
    std::vector<double> sqrLength(n);

    for(std::size_t i = 0; i < n; ++i) {
        Y[i] = P * X[i];
        sqrLength[i] = Y[i].dot(Y[i]);

        A += Y[i] * Y[i].transpose();
        B += sqrLength[i] * Y[i];

        averageSqrLength += sqrLength[i];
    }

    A /= (double) n;
    B /= (double) n;

    averageSqrLength /= n;

    Eigen::Matrix3d  Ahat = -S * A * S;
    PC = (Ahat * B) / (Ahat * A).trace();

    double error = 0.0;
    rsqr = 0.0;

    for(std::size_t i = 0; i < n; ++i) {
        double term = sqrLength[i] - averageSqrLength - 2 * Y[i].dot(PC);
        error += term * term;
        Eigen::Vector3d diff = PC - Y[i];
        rsqr += diff.dot(diff);
    }

    error /= n;
    rsqr /= n;

    return error;
}



