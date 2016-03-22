/// HEADER
#include "pillar_localization/pillar.h"


Pillar::Pillar()
    : points(0), measured_radius(0.0)
{

}

Pillar::Pillar(const Eigen::Vector3d& centre)
    : centre(centre), points(0), measured_radius(0.0)
{

}

Pillar::Pillar(const Eigen::Vector3d& centre, int points)
    : centre(centre), points(points), measured_radius(0.0)
{

}

bool Pillar::operator < (const Pillar& other) const
{
    return points < other.points;
}


bool Pillar::operator > (const Pillar& other) const
{
    return points > other.points;
}
