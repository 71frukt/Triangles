#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math/vector3.hpp"
#include <stdexcept>

#include "RLogSU/logger.hpp"

namespace Geometry::Primitives {


void Point3::DumpDetails() const
{
    RLSU_LOG("({:.5g}, {:.5g}, {:.5g})\n", GetX(), GetY(), GetZ());
}

void Line3::DumpDetails() const
{
    origin_   .Dump("origin");
    normd_dir_.Dump("normd_dir");
}

void Plane3::DumpDetails() const
{
    RLSU_LOG("{:+.5g}x {:+.5g}y {:+.5g}z {:+.5g} = 0\n", GetA(), GetB(), GetC(), GetD());
    normd_normality_.Dump("normd_normality");
}

}