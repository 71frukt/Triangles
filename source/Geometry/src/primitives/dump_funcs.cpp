#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math/vector3.hpp"
#include <stdexcept>

#include "RLogSU/logger.hpp"

namespace Geometry::Primitives {


static const char *const ObjTypeStr(ObjType type)
{
    #define CASE_TYPE(type_)  case(type_): return #type_

    switch (type)
    {
        CASE_TYPE(POINT3);
        CASE_TYPE(LINE3);
        CASE_TYPE(PLANE3);
        CASE_TYPE(NOT_AN_OBJ);

        default:
        {
            throw std::runtime_error("UNKNOWN TYPE in ObjTypeStr()");
            return "UNKNOWN TYPE!!!";
        }
    }
    #undef CASE_TYPE
}

void Point3::Dump() const
{
    RLSU_LOG("Point [{}]  (typeof {})\n", static_cast<const void*>(this), ObjTypeStr(WhoAmI()));

    RLSU_LOG("\t({}, {}, {})\n", GetX(), GetY(), GetZ());

    RLSU_LOG("\n");
}

}