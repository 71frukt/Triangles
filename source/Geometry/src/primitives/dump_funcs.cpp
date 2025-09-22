#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math/vector3.hpp"
#include <stdexcept>

#include "RLogSU/logger.hpp"

namespace Geometry::Primitives {


void Point3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("'{}' [{}]  (typeof {}) {{\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    RLSU_BASETAB_INCREACE;

    RLSU_LOG("({:.2f}, {:.2f}, {:.2f})\n", GetX(), GetY(), GetZ());
    
    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}

void Line3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("'{}' [{}]  (typeof {}) {{\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    RLSU_BASETAB_INCREACE;

    origin_   .Dump("origin");
    normd_dir_.Dump("normd_dir");


    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}

void Plane3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("'{}' [{}]  (typeof {}) {{\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    RLSU_BASETAB_INCREACE;

    RLSU_LOG("{:+.2f}x {:+.2f}y {:+.2f}z {:+.2f} = 0\n", GetA(), GetB(), GetC(), GetD());
    normd_normality_.Dump("normd_normality");
    
    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}

}