
#include "Geometry/primitives/primitives.hpp"
#include "RLogSU/logger.hpp"


int main()
{
    RLSU_INFO("START");

    Geometry::Primitives::Plane3 plane(0, 0, 2, 1);

    plane.Dump("myplane");

    Geometry::Primitives::Line3 line({0, 1, 2}, {6, -6, 3});

    line.Dump("myline");

    RLSU_INFO("END");
}