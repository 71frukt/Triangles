#pragma once

#include "Geometry/math/point3.hpp"
#include "Geometry/primitives/primitive.hpp"


namespace Geometry::Primitives {


class Point3 : public Geometry::Math::Point3
             , Primitive
{
public:
    [[nodiscard]] virtual Type WhoAmI() const override final { return Type::POINT3; };      
private:

};

}