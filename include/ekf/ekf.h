#ifndef EKF_H
#define EKF_H

#include "pillar_localization/pillar.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

struct EKF {
    EKF();

    void reset();

    void setPillars(const std::vector<Pillar>& pillars);

    void predict(const Eigen::Vector3d &delta,
                 double v, double omega, double dt);

    void correct(const std::vector<Pillar> &z);
    void correctLandmark(const std::vector<Pillar> &z);
    bool correctAbsolute(const std::vector<Pillar> &z, bool fix = false);


    Eigen::Vector3d mu;
    Eigen::Matrix3d P;

    Eigen::Matrix3d I;

    Eigen::Matrix3d G;
    Eigen::Matrix3d R;

    Eigen::Matrix3d Q;
    Eigen::Matrix3d Q_abs;

    double dist_threshold_;

    std::vector<Pillar> pillars_;
    std::vector<Pillar> meas_pillars_;

    double dist_1_;
    double dist_2_;
    double dist_3_;

    bool initialized_;
    Eigen::Matrix3d initial_pose_;
    Eigen::Matrix3d initial_pose_inv_;
    double initial_pose_yaw_;


private:
    void updateMeasurement(const std::vector<Pillar>& z);
};

#endif // EKF_H
