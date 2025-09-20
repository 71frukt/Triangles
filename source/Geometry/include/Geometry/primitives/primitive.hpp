#pragma once

#include "Geometry/common/geometry_obj.hpp"

namespace Geometry::Primitives {

class Primitive : GeometryObj
{
public:
    virtual ~Primitive() = default;

    enum class Type
    {
        POINT3,
        LINE3,
        PLANE3,
    };

    [[nodiscard]] virtual Type WhoAmI() const = 0;

private:
};


}