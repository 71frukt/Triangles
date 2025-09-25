
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/shapes/shapes.hpp"

#include <cstddef>
#include <functional>

namespace Geometry::MathEngine {


/*
Implemented interactions table

---------------------------------------------------------
|          || point | line | plane | linesect | triangle
-----------++--------------------------------------------
| point    ||   1   |   1  |   1   |    1     |    0
| line     ||       |   1  |   1   |    1     |    1
| plane    ||       |      |   1   |    0     |    0
| linesect ||       |      |       |    1     |    0
| triangle ||       |      |       |          |    1
---------------------------------------------------------
*/

std::unique_ptr<Interactor> Interact(const GeomObj& obj1, const GeomObj& obj2)
{
    using InteractorCreator = std::function<std::unique_ptr<Interactor>(const GeomObj&, const GeomObj&)>;

    static std::array<std::array<InteractorCreator, OBJ_TYPES_NUM>, OBJ_TYPES_NUM> interactor_table =[]()
    {
        std::array<std::array<InteractorCreator, OBJ_TYPES_NUM>, OBJ_TYPES_NUM> table ={};

        // initially, everything is NotAnObjInteractor

        for (size_t i = 0; i < OBJ_TYPES_NUM; i++)
        {
            for (size_t j = 0; j < i; j++)
            {
                table[i][j] = [](const GeomObj& obj1, const GeomObj& obj2) {
                    return std::make_unique<NotImplementedInteractor>(obj1, obj2);
                };
            }
        }

        // here we define it correctly. 
        // what is undefined remains the NotAnObjInteractor. Later, when all the interactors are written, 
        // this crutch will go away, but so far (and most likely in this implementation) all the interactors 
        // of all objects will not be completed, so to avoid stupid mistakes, we do this


        for (size_t i = 0; i < OBJ_TYPES_NUM; i++)
        {
            table[NOT_AN_OBJ][i] = [](const GeomObj& obj1, const GeomObj& obj2) {
                    return std::make_unique<NotAnObjInteractor>(obj1, obj2);
                };

            table[i][NOT_AN_OBJ] = [](const GeomObj& obj1, const GeomObj& obj2) {
                    return std::make_unique<NotAnObjInteractor>(obj1, obj2);
                };
        }

        // =====================================================================

        table[POINT3][POINT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointPointInteractor>(
                    static_cast<const Primitives::Point3&>(obj1),
                    static_cast<const Primitives::Point3&>(obj2)
                ); 
            };

        // =====================================================================

        table[POINT3][PLANE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointPlaneInteractor>(
                    static_cast<const Primitives::Point3&>(obj1),
                    static_cast<const Primitives::Plane3&>(obj2)
                ); 
            };

        table[PLANE3][POINT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointPlaneInteractor>(
                    static_cast<const Primitives::Point3&>(obj2),
                    static_cast<const Primitives::Plane3&>(obj1)
                ); 
            };

        // =====================================================================

        table[POINT3][LINE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointLineInteractor>(
                    static_cast<const Primitives::Point3&>(obj1),
                     static_cast<const Primitives::Line3 &>(obj2)
                ); 
            };
        
        table[LINE3][POINT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointLineInteractor>(
                    static_cast<const Primitives::Point3&>(obj2),
                     static_cast<const Primitives::Line3 &>(obj1)
                ); 
            };

        // =====================================================================

        table[LINE3][LINE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LineLineInteractor>(
                    static_cast<const Primitives::Line3&>(obj1),
                    static_cast<const Primitives::Line3&>(obj2)
                ); 
            };
        
        // =====================================================================

        table[LINE3][PLANE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LinePlaneInteractor>(
                     static_cast<const Primitives::Line3 &>(obj1),
                    static_cast<const Primitives::Plane3&>(obj2)
                ); 
            };

        table[PLANE3][LINE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LinePlaneInteractor>(
                     static_cast<const Primitives::Line3 &>(obj2),
                    static_cast<const Primitives::Plane3&>(obj1)
                ); 
            };
        // =====================================================================

        table[PLANE3][PLANE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PlanePlaneInteractor>(
                    static_cast<const Primitives::Plane3&>(obj1),
                    static_cast<const Primitives::Plane3&>(obj2)
                ); 
            };
        // =====================================================================
        
        table[POINT3][LINESECT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointLinesectInteractor>(
                       static_cast<const Primitives::Point3&>(obj1),
                    static_cast<const Shapes::Linesect3 &>(obj2)
                ); 
            };

        table[LINESECT3][POINT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointLinesectInteractor>(
                       static_cast<const Primitives::Point3&>(obj2),
                    static_cast<const Shapes::Linesect3 &>(obj1)
                ); 
            };

        // =====================================================================

        table[LINE3][LINESECT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LineLinesectInteractor>(
                        static_cast<const Primitives::Line3&>(obj1),
                    static_cast<const Shapes::Linesect3&>(obj2)
                ); 
            };

        table[LINESECT3][LINE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LineLinesectInteractor>(
                        static_cast<const Primitives::Line3&>(obj2),
                    static_cast<const Shapes::Linesect3&>(obj1)
                ); 
            };

        // =====================================================================

        table[LINESECT3][LINESECT3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LinesectLinesectInteractor>(
                    static_cast<const Shapes::Linesect3&>(obj1),
                    static_cast<const Shapes::Linesect3&>(obj2)
                ); 
            };

        // =====================================================================

        table[LINE3][TRIANGLE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LineTriangleInteractor>(
                        static_cast<const Primitives::Line3&>(obj1),
                    static_cast<const Shapes::Triangle3&>(obj2)
                ); 
            };

        table[TRIANGLE3][LINE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LineTriangleInteractor>(
                        static_cast<const Primitives::Line3&>(obj2),
                    static_cast<const Shapes::Triangle3&>(obj1)
                ); 
            };
        // =====================================================================

        table[TRIANGLE3][TRIANGLE3] =
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<TriangleTriangleInteractor>(
                    static_cast<const Shapes::Triangle3&>(obj1),
                    static_cast<const Shapes::Triangle3&>(obj2)
                ); 
            };

        return table;
    }();

    return interactor_table[obj1.WhoAmI()][obj2.WhoAmI()](obj1, obj2);
}

} // namespace Geometry/MathEngine