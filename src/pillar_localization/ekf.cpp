/// HEADER
#include "ekf.h"

/// SYSTEM
#include <stdexcept>
#include <Eigen/Dense>

EKF::EKF()
    : initialized_(false)
{
    mu << 0, 0, 0;

    I = Eigen::Matrix3d::Identity();

    P = I * pow(1.0, 2);

    R = Eigen::Matrix3d::Identity();
    R(0,0) = pow(0.15, 2);
    R(1,1) = pow(0.15, 2);
    R(2,2) = pow(0.1, 2);

    //    Q =  Eigen::Matrix3d::Identity() * pow(10.0, 2);
    Q = Eigen::Matrix2d::Identity();
    Q(0,0) = pow(0.3, 2);
    Q(1,1) = pow(0.3, 2);

    Q_abs = Eigen::Matrix3d::Identity();
    Q_abs(0,0) = pow(0.6, 2);
    Q_abs(1,1) = pow(0.6, 2);
    Q_abs(2,2) = pow(0.15, 2);


    dist_threshold_ = 1.0;
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

    initialized_ = false;
}

void EKF::predict(const Eigen::Vector3d& delta, double _v, double _omega, double dt)
{
    double theta = mu(2);

    double f = 0.7;

    double omega = f * _omega;
    double v = f * _v;

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
}

void EKF::correct(const std::vector<Pillar>& z)
{
    std::size_t N = pillars_.size();

    typedef Eigen::Matrix<double, 2, 3> Matrix23;
    typedef Eigen::Matrix2d Matrix2;
    typedef Eigen::Matrix<double, 3, 2> Matrix32;

    std::vector<Matrix2, Eigen::aligned_allocator<Matrix2> > Psi_inv(N);
    std::vector<Matrix23, Eigen::aligned_allocator<Matrix23> > H(N);
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > z_hat(N);

    for(std::size_t k = 0; k < N; ++k) {
        const Eigen::Vector3d& m_k = pillars_[k].centre;

        double dx = m_k(0) - mu(0);
        double dy = m_k(1) - mu(1);


        double q_k = dx * dx + dy * dy;

        z_hat[k] << sqrt(q_k), std::atan2(dy, dx) - mu(2);

        H[k] << dx / std::sqrt(q_k), - dy / std::sqrt(q_k), 0,
                dy / q_k, dx / q_k, -1 / q_k;

        Psi_inv[k] = (H[k] * P * H[k].transpose() + Q).inverse();
    }


    std::vector<Matrix32, Eigen::aligned_allocator<Matrix32> > K(z.size());

    Eigen::Vector3d dmu;
    Eigen::Matrix3d dP;

    meas_pillars_.clear();

    for(std::size_t i = 0; i < z.size(); ++i) {
        const Eigen::Vector3d& p_i = z[i].centre;

        meas_pillars_.push_back(Pillar(p_i + mu));

        Eigen::Vector2d z_i;
        z_i << hypot(p_i(1), p_i(0)), std::atan2(p_i(1), p_i(0));

        double min_dist = std::numeric_limits<double>::infinity();
        std::size_t ji = 0;
        for(std::size_t k = 0; k < N; ++k) {
            Eigen::Vector2d dz = z_i - z_hat[k];
            double dist = std::abs(dz.transpose() * Psi_inv[k] * dz);
            if(dist < min_dist) {
                min_dist = dist;
                ji = k;
            }
        }

        if(std::isfinite(min_dist) && min_dist < dist_threshold_) {
            K[i] = P * H[ji].transpose() * Psi_inv[ji];

            dmu += K[i] * (z_i - z_hat[ji]);
            dP += K[i] * H[ji];
        }
    }

    mu += dmu;
    P = (I - dP) * P;
}


void EKF::correctAbsolute(const std::vector<Pillar>& z)
{
    if(z.size() >= 3) {
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
            lever = (d_ac < d_bc) ? a : b;
        } else if(d_ac == mind) {
            anchor = b;
            lever = (d_ab < d_bc) ? a : c;
        } else {
            anchor = a;
            lever = (d_ab < d_ac) ? b : c;
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
            double yaw_correction = std::atan2(base_line.y(), base_line.x());

            Eigen::Vector3d origin(anchor.x(), anchor.y(), 0);
            //            Eigen::Quaterniond quat; quat = Eigen::AngleAxis<double>(yaw_correction, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d pose;
            pose << std::cos(yaw_correction), -std::sin(yaw_correction), origin.x(),
                    std::sin(yaw_correction), std::cos(yaw_correction), origin.y(),
                    0, 0, 1;

            pose = pose.inverse().eval();

            if(!initialized_) {
                initial_pose_ = pose;
                initial_pose_inv_ = pose.inverse();
                initial_pose_yaw_ = yaw_correction;

                initialized_ = true;

            } else {
                Eigen::Matrix3d rel_pose = initial_pose_inv_ * pose;
                double yaw = yaw_correction - initial_pose_yaw_;

                Eigen::Vector3d p = rel_pose.col(2);

                Eigen::Vector3d z(p(0), p(1), -yaw);
                Eigen::Vector3d hx = mu;

                Eigen::Matrix3d H = I;

                Eigen::Matrix3d K = P * H.transpose() * (H * P * H.transpose() + Q_abs).inverse();

                mu = mu + K * (z - hx);
                P = (I - K * H) * P;
            }
        }
    }
}

