#ifndef PILLAR_H
#define PILLAR_H

/// SYSTEM
#include <Eigen/Core>

struct Pillar
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pillar();
    Pillar(const Eigen::Vector3d& centre);
    Pillar(const Eigen::Vector3d& centre, int points);

    bool operator < (const Pillar& other) const;
    bool operator > (const Pillar& other) const;

public:
    Eigen::Vector3d centre;
    Eigen::Vector3d up;
    int points;
    double measured_radius;
};

#endif // PILLAR_H
