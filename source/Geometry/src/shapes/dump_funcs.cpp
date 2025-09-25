#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/logger.hpp"

namespace Geometry::Shapes {

void Linesect3::DumpDetails() const
{
    line_.Dump("line");

    point1_.Dump("point1");
    point2_.Dump("point2");
}

// void Halfint3::DumpDetails() const
// {
//     line_.Dump("line");

//     close_end_.Dump("close_end");
//     open_end_ .Dump("open_end");
// }

void Triangle3::DumpDetails() const
{
    point1_.Dump("point1");
    point2_.Dump("point2");
    point3_.Dump("point3");

    side1_ .Dump("side1" );
    side2_ .Dump("side2" );
    side3_ .Dump("side3" );

    plane_ .Dump("plane" );
}

}