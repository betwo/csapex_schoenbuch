/// HEADER
#include "ekf/ekf.h"

/// SYSTEM
#include <stdexcept>
#include <Eigen/Dense>
#include <ros/console.h>

namespace {
double angleDiff(const double a1, const double a2)
{
    return std::atan2(std::sin(a1 - a2), std::cos(a1 - a2)) ;
}
double normalizeAngle(const double a)
{
    double res = a;
    while (res <= -M_PI) {
        res += 2 * M_PI;
    }
    while (res > M_PI) {
        res -= 2 * M_PI;
    }
    return res;
}
}

EKF::EKF()
    : initialized_(false)
{
    I = Eigen::Matrix3d::Identity();

    R = Eigen::Matrix3d::Identity();
    R(0,0) = pow(0.1, 2);
    R(1,1) = pow(0.1, 2);
    R(2,2) = pow(0.3, 2);

    //    Q =  Eigen::Matrix3d::Identity() * pow(10.0, 2);
    Q = Eigen::Matrix3d::Identity();
    Q(0,0) = pow(0.35, 2);
    Q(1,1) = pow(0.35, 2);
    Q(2,2) = pow(0.5, 2);

    Q_abs = Eigen::Matrix3d::Identity();
    Q_abs(0,0) = pow(0.05, 2);
    Q_abs(1,1) = pow(0.05, 2);
    Q_abs(2,2) = pow(0.001, 2);


    dist_threshold_ = 1.0;

    reset();
}

void EKF::reset()
{
    initialized_ = false;

    mu << 0, 0, 0;
    P = I * pow(1.0, 2);
}

void EKF::setPillars(const std::vector<Pillar>& pillars)
{
    if(pillars.size() != 3) {
        throw std::runtime_error("cannot initialize, need exactly 3 pillar candiates");
    }

    pillars_ = pillars;

    Eigen::Vector3d pillar_a = pillars_[0].centre;
    Eigen::Vector3d pillar_b = pillars_[1].centre;
    Eigen::Vector3d pillar_c = pillars_[2].centre;

    Eigen::Vector3d ab = pillar_b - pillar_a;
    Eigen::Vector3d ac = pillar_c - pillar_a;
    Eigen::Vector3d bc = pillar_c - pillar_b;

    dist_1_ = (ac).norm();
    dist_2_ = (ab).norm();
    dist_3_ = (bc).norm();

    Eigen::Matrix3d pose;
    if(!findAbsolutePose(pillars, pose)) {
        throw std::runtime_error("cannot initialize, pillars are not detected correctly");
    }

    initial_pose_ = pose;
    initial_pose_inv_ = pose.inverse();

    Eigen::Matrix2d rot = pose.block<2,2>(0,0);
    Eigen::Vector2d x = rot * Eigen::Vector2d(1,0);
    mu(2) = std::atan2(x(1), x(0));

    mu(0) = pose(0,2);
    mu(1) = pose(1,2);

    initialized_ = true;
}

bool EKF::predict(const Eigen::Vector3d& delta, double _v, double _omega, double dt)
{
    double theta = mu(2);

    double omega = delta(2) / dt;
    double v = delta(0) / dt;

    if(std::abs(_v) < 1e-1 && std::abs(_omega) < 1e-1) {
        return false;
    }

    if(omega == 0.0) {
        omega = 1e-10;
    }

    Eigen::Vector3d g;
//    g <<    v / omega * (sin(theta + omega * dt) - sin(theta)),
//            v / omega * (cos(theta) - cos(theta + omega * dt)),
//            omega * dt;
    g << std::cos(theta) * delta(0) - std::sin(theta) * delta(1),
            std::sin(theta) * delta(0) + std::cos(theta) * delta(1),
            delta(2);

    G <<    1, 0, v / omega * (cos(theta) - cos(theta + omega * dt)),
            0, 1, v / omega * (sin(theta) - sin(theta + omega * dt)),
            0, 0, 1;

    mu += g;

    P = G * P * G.transpose() + R;

    mu(2) = normalizeAngle(mu(2));

    return true;
}


void EKF::correct(const std::vector<Pillar>& z)
{
    if(z.size() >= 3) {
        correctAbsolute(z);
    } else {
        correctLandmark(z);
    }

    updateMeasurement(z);
}

void EKF::updateMeasurement(const std::vector<Pillar>& z)
{
    meas_pillars_.clear();

    Eigen::Matrix4d fixed_t_robot;
    fixed_t_robot << std::cos(mu(2)), -std::sin(mu(2)), 0, mu(0),
            std::sin(mu(2)), std::cos(mu(2)), 0, mu(1),
            0, 0, 1, 0,
            0, 0, 0, 1;

    for(std::size_t i = 0; i < std::min((std::size_t) 3, z.size()); ++i) {
        Eigen::Vector4d p_i_robot;
        p_i_robot.block<3,1>(0,0) = z[i].centre;
        p_i_robot(3,0) = 1;

        Eigen::Vector4d p_i = fixed_t_robot * p_i_robot;

        meas_pillars_.push_back(p_i.block<3,1>(0,0).eval());
    }
}

void EKF::correctLandmark(const std::vector<Pillar>& z)
{
    std::size_t N = pillars_.size();

    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > S(N);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > H(N);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > z_hat(N);

    for(std::size_t k = 0; k < N; ++k) {
        const Eigen::Vector3d& m_k = pillars_[k].centre;

        double dx = m_k(0) - mu(0);
        double dy = m_k(1) - mu(1);


        double q_k = dx * dx + dy * dy;

        z_hat[k] << sqrt(q_k), normalizeAngle(std::atan2(dy, dx) - mu(2)), 0;

        H[k] << -dx / std::sqrt(q_k), - dy / std::sqrt(q_k), 0,
                dy / q_k, -dx / q_k, -1,
                0, 0, 0;

        S[k] = H[k] * P * H[k].transpose() + Q;
    }


    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > K(z.size());

    for(std::size_t i = 0; i < z.size(); ++i) {
        Eigen::Vector3d p_i = z[i].centre;

        Eigen::Vector3d z_i;
        z_i << hypot(p_i(1), p_i(0)), std::atan2(p_i(1), p_i(0)), 0;

        double max_p = -std::numeric_limits<double>::infinity();
        std::size_t ji = 0;
        for(std::size_t k = 0; k < N; ++k) {
            Eigen::Vector3d dz = z_i - z_hat[k];
            //            dz(2) = angleDiff(z_i(2), z_hat[k](2));
            double norm = std::sqrt((2 * M_PI * S[k]).determinant());
            double p = norm * std::exp(-0.5 * dz.transpose() * S[k].inverse() * dz);
            if(p > max_p) {
                max_p = p;
                ji = k;
            }
        }

        if(std::isfinite(max_p) && max_p > 0.5) {
            K[i] = P * H[ji].transpose() * S[ji].inverse();

            Eigen::Vector3d dz = z_i - z_hat[ji];
            //            dz(2) = angleDiff(z_i(2), z_hat[ji](2));

            ROS_WARN_STREAM("z_i:  " << z_i);
            ROS_WARN_STREAM("z_ih: " <<  z_hat[ji]);
            ROS_WARN_STREAM("dz: " << dz << ", max_p:" << max_p);

            mu = mu + K[i] * dz;
            P = (I - K[i] * H[ji]) * P;
        }
    }

    mu(2) = normalizeAngle(mu(2));

}

bool EKF::correctAbsolute(const std::vector<Pillar>& pillars, bool fix)
{
    if(!initialized_) {
        return false;
    }

    Eigen::Matrix3d pose;

    if(!findAbsolutePose(pillars, pose)) {
        return false;
    }

    Eigen::Matrix3d rel_pose = initial_pose_inv_ * pose;
    Eigen::Matrix2d rot = rel_pose.block<2,2>(0,0);

    Eigen::Vector2d x = rot * Eigen::Vector2d(1,0);
    double yaw = std::atan2(x(1), x(0));

    Eigen::Vector3d p = rel_pose.col(2);
    Eigen::Vector3d z(p(0), p(1), yaw);

    if(fix) {
        mu = z;
        P = Q_abs;

    } else {
        Eigen::Vector3d hx = mu;

        Eigen::Matrix3d H = I;

        Eigen::Matrix3d K = P * H.transpose() * (H * P * H.transpose() + Q_abs).inverse();

        Eigen::Vector3d innovation = z - hx;
        // special case -> angle wrap
        innovation(2) = angleDiff(z(2), hx(2));

        mu = mu + K * innovation;
        mu(2) = normalizeAngle(mu(2));

        P = (I - K * H) * P;
    }

    return true;
}

bool EKF::findAbsolutePose(const std::vector<Pillar>& z, Eigen::Matrix3d &pose)
{
    assert(z.size() >= 3);

    const Eigen::Vector3d& a = z[0].centre;
    const Eigen::Vector3d& b = z[1].centre;
    const Eigen::Vector3d& c = z[2].centre;

    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d ac = c - a;
    Eigen::Vector3d bc = c - b;

    double d_ac = (ac).norm();
    double d_ab = (ab).norm();
    double d_bc = (bc).norm();

    double mind = std::min(d_ac, std::min(d_ab, d_bc));

    Eigen::Vector3d anchor;
    Eigen::Vector3d lever;
    if(d_ab == mind) {
        anchor = c;
        lever = (d_ac > d_bc) ? a : b;
    } else if(d_ac == mind) {
        anchor = b;
        lever = (d_ab > d_bc) ? a : c;
    } else {
        anchor = a;
        lever = (d_ab > d_ac) ? b : c;
    }

    std::vector<double> dists_meas;
    dists_meas.push_back(d_ac);
    dists_meas.push_back(d_ab);
    dists_meas.push_back(d_bc);
    std::vector<double> dists_ref;
    dists_ref.push_back(dist_1_);
    dists_ref.push_back(dist_2_);
    dists_ref.push_back(dist_3_);

    std::sort(dists_meas.begin(), dists_meas.end());
    std::sort(dists_ref.begin(), dists_ref.end());

    double max_delta = 0;
    for(std::size_t i = 0; i < 3; ++i) {
        double delta = std::abs(dists_meas[i] - dists_ref[i]);
        if(delta > max_delta) {
            max_delta = delta;
        }
    }

    if(max_delta < dist_threshold_) {
        Eigen::Vector3d base_line = lever - anchor;
        double yaw = -std::atan2(base_line.y(), base_line.x());

        Eigen::Vector3d origin(std::cos(yaw) * anchor.x() - std::sin(yaw) * anchor.y(),
                               std::sin(yaw) * anchor.x() + std::cos(yaw) * anchor.y(),
                               0);
        pose << std::cos(yaw), -std::sin(yaw), -origin(0),
                std::sin(yaw), std::cos(yaw), -origin(1),
                0, 0, 1;

        return true;

    } else {
        return false;
    }
}
