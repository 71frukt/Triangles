
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/collision_handler.hpp"

#include <functional>

namespace Geometry::MathEngine {


std::unique_ptr<Interactor> Interact(const GeomObj& obj1, const GeomObj& obj2)
{
    using InteractorCreator = std::function<std::unique_ptr<Interactor>(const GeomObj&, const GeomObj&)>;

    static std::array<std::array<InteractorCreator, 5>, 5> interactor_table =[]()
    {
        std::array<std::array<InteractorCreator, 5>, 5> table ={};

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

        return table;
    }();

    return interactor_table[obj1.WhoAmI()][obj2.WhoAmI()](obj1, obj2);
}

} // namespace Geometry/MathEngine