/// HEADER
#include "ekf.h"

/// SYSTEM
#include <Eigen/Dense>

EKF::EKF()
{
    mu << 0, 0, 0;

    I =  Eigen::Matrix3d::Identity();

    P =  I * pow(1.0, 2);

    R(0,0) = pow(0.05, 2);
    R(1,1) = pow(0.05, 2);
    R(1,1) = pow(0.1, 2);

//    Q =  Eigen::Matrix3d::Identity() * pow(10.0, 2);
    Q(0,0) = pow(0.3, 2);
    Q(1,1) = pow(0.3, 2);


    dist_threshold_ = 1.0;
}

void EKF::setPillars(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& pillars)
{
    pillars_ = pillars;
}

void EKF::predict(const tf::Pose &delta, double _v, double _omega, double dt)
{
    double theta = mu(2);

    double f = 0.7;

    double omega = f * _omega;
    double v = f * _v;

    Eigen::Vector3d g;
    g <<    v / omega * (sin(theta + omega * dt) - sin(theta)),
            v / omega * (cos(theta) - cos(theta + omega * dt)),
            omega * dt;

//    double ox = delta.getOrigin().x();
//    double oy = delta.getOrigin().y();

//    g << cos(theta) * ox - sin(theta) * oy,
//            sin(theta) * ox + cos(theta) * oy,
//            tf::getYaw(delta.getRotation());
//    g <<    v * cos(theta) * dt,
//            v * sin(theta) * dt,
//            omega * dt * f;

    G <<    1, 0, v / omega * (cos(theta) - cos(theta + omega * dt)),
            0, 1, v / omega * (sin(theta) - sin(theta + omega * dt)),
            0, 0, 1;

    mu += g;
    P = G * P * G.transpose() + R;
}

void EKF::correct(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& z)
{
//    tf::Transform inv_pose = pose.inverse();
//    mu(0) = inv_pose.getOrigin().x();
//    mu(1) = inv_pose.getOrigin().y();
//    mu(2) = tf::getYaw(inv_pose.getRotation());

    std::size_t N = pillars_.size();

    typedef Eigen::Matrix<double, 2, 3> Matrix23;
    typedef Eigen::Matrix2d Matrix2;
    typedef Eigen::Matrix<double, 3, 2> Matrix32;

    std::vector<Matrix2, Eigen::aligned_allocator<Matrix2> > Psi_inv(N);
    std::vector<Matrix23, Eigen::aligned_allocator<Matrix23> > H(N);
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > z_hat(N);

    for(std::size_t k = 0; k < N; ++k) {
        const Eigen::Vector3d& m_k = pillars_[k];

        double dx = m_k(0) - mu(0);
        double dy = m_k(1) - mu(1);

        double q_k = dx * dx + dy * dy;

        z_hat[k] << sqrt(q_k), std::atan2(dy, dx) - mu(2);

        H[k] << std::sqrt(q_k) * dx, - std::sqrt(q_k) * dy, 0,
                dy, dx, -1;
        H[k] *= 1 / q_k;

        Psi_inv[k] = (H[k] * P * H[k].transpose() + Q).inverse();
    }


    std::vector<Matrix32, Eigen::aligned_allocator<Matrix32> > K(z.size());

    Eigen::Vector3d dmu;
    Eigen::Matrix3d dP;

    for(std::size_t i = 0; i < z.size(); ++i) {
        const Eigen::Vector3d& p_i = z[i];

        Eigen::Vector2d z_i;
        z_i << hypot(p_i(1), p_i(0)), std::atan2(p_i(1), p_i(0));

        double min_dist = std::numeric_limits<double>::infinity();
        std::size_t ji = 0;
        for(std::size_t k = 0; k < N; ++k) {
            Eigen::Vector2d dz = z_i - z_hat[k];
            double dist = std::abs(dz.transpose() * Psi_inv[k] * dz);
            ROS_WARN_STREAM("dist: " << dist << ", z_i:\n" << z_i << ", zihat:\n" << z_hat[k]);
            if(dist < min_dist) {
                min_dist = dist;
                ji = k;
            }
        }

        if(std::isfinite(min_dist) && min_dist < dist_threshold_) {

            ROS_INFO_STREAM("min dist: " << min_dist << ", j(i): " << ji);

            K[i] = P * H[ji].transpose() * Psi_inv[ji];

            ROS_WARN_STREAM("P:\n" << P << " *\n" << H[ji].transpose() << " *\n" << Psi_inv[ji]);
            ROS_WARN_STREAM("i: " << i << ", Ki:\n" << K[i]);

            dmu += K[i] * (z_i - z_hat[ji]);
            dP += K[i] * H[ji];

        } else {
            ROS_WARN_STREAM("min dist: " << min_dist);
        }
    }


    ROS_ERROR_STREAM("dmu:\n" << dmu);
    ROS_ERROR_STREAM("dP:\n" << (-dP*P));

//    mu += dmu;
    P = (I - dP) * P;
}
