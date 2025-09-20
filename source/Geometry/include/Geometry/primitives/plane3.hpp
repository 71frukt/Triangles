#pragma once

#include "Geometry/primitives/primitive.hpp"
#include "Geometry/primitives/point3.hpp"
#include "Geometry/math/vector3.hpp"

namespace Geometry::Primitives {

class Plane3 : Primitive
{
public:

    Plane3(const Math::Vector3& normality_, const double D) : normalized_normality_(normality_.Normalized()), D_(D) {};
    Plane3(const double A, const double B, const double C, const double D);
    Plane3(Point3 A, Point3 B, Point3 C);

    [[nodiscard]] virtual Type WhoAmI() const override final { return Type::PLANE3; };      

private:
    Math::Vector3 normalized_normality_;
    double  D_;
};

}