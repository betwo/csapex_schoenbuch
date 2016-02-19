#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>
#include <tf/tf.h>

struct EKF {
    EKF();

    void setPillars(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& pillars);

    void predict(const tf::Pose& delta,
                 double v, double omega, double dt);

    void correct(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &z);

    Eigen::Vector3d mu;
    Eigen::Matrix3d P;

    Eigen::Matrix3d I;

    Eigen::Matrix3d G;
    Eigen::Matrix3d R;

    Eigen::Matrix2d Q;

    double dist_threshold_;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pillars_;
};

#endif // EKF_H
