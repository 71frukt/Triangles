
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/logger.hpp"
#include <cstddef>
#include <iostream>
#include <ostream>
#include <vector>
#include <set>


int main()
{
    size_t N = 0;
    if (!(std::cin >> N))
    {
        RLSU_ERROR("failed to read N");
        return 666;
    }

    std::vector<Geometry::GeomObjUniqPtr> triangles(N);
    std::vector<const Geometry::GeomObj*> triangles_ptrs(N);

    for (size_t i = 0; i < N; i++)
    {
        Geometry::Primitives::Point3 point1;
        Geometry::Primitives::Point3 point2;
        Geometry::Primitives::Point3 point3;

        double var = 0;

        if (std::cin >> var) point1.SetX(var);  else RLSU_ERROR("error reading point axis");
        if (std::cin >> var) point1.SetY(var);  else RLSU_ERROR("error reading point axis");
        if (std::cin >> var) point1.SetZ(var);  else RLSU_ERROR("error reading point axis");

        if (std::cin >> var) point2.SetX(var);  else RLSU_ERROR("error reading point axis");
        if (std::cin >> var) point2.SetY(var);  else RLSU_ERROR("error reading point axis");
        if (std::cin >> var) point2.SetZ(var);  else RLSU_ERROR("error reading point axis");
        
        if (std::cin >> var) point3.SetX(var);  else RLSU_ERROR("error reading point axis");
        if (std::cin >> var) point3.SetY(var);  else RLSU_ERROR("error reading point axis");
        if (std::cin >> var) point3.SetZ(var);  else RLSU_ERROR("error reading point axis");

        triangles[i]      = Geometry::Shapes::Triangle3::BuildGeomObj(point1, point2, point3);
        triangles_ptrs[i] = triangles[i].get();
    }


    Geometry::MathEngine::BvhTree bvh_tree(triangles_ptrs);

    RLSU_DUMP(bvh_tree.Dump());

    auto intersections = bvh_tree.GetIntersections();

    RLSU_INFO("--------RESULT--------" std::endl);

    for (size_t i = 0; i < intersections.size(); i++)
    {
        if (intersections[i] == true)
        {
            std::cout << i << std::endl;
        }
    }

}