#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math/vector3.hpp"

#include "RLogSU/logger.hpp"

namespace Geometry::Primitives {

Plane3::Plane3(const double A, const double B, const double C, const double D)
{
    Math::Vector3 normality(A, B, C);

    normd_normality_ = normality.Normalized();
    D_ = D / normality.GetLen();
}


Plane3::Plane3(Point3 A, Point3 B, Point3 C)
{
    Math::Vector3 ab = B - A;
    Math::Vector3 ac = B - C;
    
    normd_normality_ = (ab ^ ac).Normalized();

    Math::Vector3 rad_vec_0(A);
    D_ = -(rad_vec_0 * normd_normality_);
}

}