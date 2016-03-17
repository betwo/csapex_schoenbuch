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
    int points;
};

#endif // PILLAR_H
