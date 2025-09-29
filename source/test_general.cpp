
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"
#include <cstddef>
#include <iostream>
#include <ostream>


int main()
{
    size_t N = 0;
    std::cin >> N;

    Geometry::GeomObjUniqPtr triangles[N];

    bool cross[N][N];
    for (size_t i = 0; i < N; i++)
        for (size_t j = 0; j < N; j++)
            cross[i][j] = false;


    for (size_t i = 0; i < N; i++)
    {
        Geometry::Primitives::Point3 point1;
        Geometry::Primitives::Point3 point2;
        Geometry::Primitives::Point3 point3;

        double var = 0;

        std::cin >> var;    point1.SetX(var);
        std::cin >> var;    point1.SetY(var);
        std::cin >> var;    point1.SetZ(var);

        std::cin >> var;    point2.SetX(var);
        std::cin >> var;    point2.SetY(var);
        std::cin >> var;    point2.SetZ(var);

        std::cin >> var;    point3.SetX(var);
        std::cin >> var;    point3.SetY(var);
        std::cin >> var;    point3.SetZ(var);

        triangles[i] = Geometry::Shapes::Triangle3::BuildGeomObj(point1, point2, point3);
    }

    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j < N; j++)
        {
            if (Geometry::MathEngine::Interact(*triangles[i], *triangles[j])->CollisionCode() != Geometry::MathEngine::NOTHING)
                cross[i][j] = true;

            else
                cross[i][j] = false;
        }
    }

    for (size_t i = 0; i < N; i++)
    {
        bool crosses = false;

        for (size_t j = 0; j < N; j++)
        {
            if (i == j)
                continue;
            
            if (cross[i][j] == true)
            {
                crosses = true;
                break;
            }
        }

        if (crosses)
            std::cout << i << " ";
    }

    std::cout << std::endl;
}