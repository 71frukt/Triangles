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

void Point3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("{} [{}]  (typeof {} {{\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    RLSU_BASETAB_INCREACE;

    RLSU_LOG("({}, {}, {})\n", GetX(), GetY(), GetZ());
    
    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}

void Line3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("{} [{}]  (typeof {}) {{\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    RLSU_BASETAB_INCREACE;

    origin_.Dump("origin");
    normalized_director_.Dump("normd_dir");


    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}

void Plane3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("{} [{}]  (typeof {}) {{\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    RLSU_BASETAB_INCREACE;

    RLSU_LOG("{}x + {}y + {}z + {} = 0\n", GetA(), GetB(), GetC(), GetD());
    normd_normality_.Dump("normd_normality");
    
    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}

}