#pragma once

#include "Geometry/math/vector3.hpp"
#include "Geometry/primitives/primitive.hpp"
#include "Geometry/primitives/point3.hpp"

namespace Geometry::Primitives {

class Line3 : Primitive
{
public:
    Line3(Point3 origin, Math::Vector3 director) : origin_(origin), normalized_director_(director.Normalized()) {}

    [[nodiscard]] virtual Type WhoAmI() const override final { return Type::LINE3; };
    
private:
    Point3  origin_;
    Math::Vector3 normalized_director_;
};

}