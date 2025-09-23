
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/collision_handler.hpp"

namespace Geometry::MathEngine {


std::unique_ptr<Interactor> Interact(const GeomObj& obj1, const GeomObj& obj2)
{
    using InteractorCreator = std::function<std::unique_ptr<Interactor>(const GeomObj&, const GeomObj&)>;

    static std::array<std::array<InteractorCreator, 4>, 4> interactor_table = []()
    {
        std::array<std::array<InteractorCreator, 4>, 4> table = {};

        table[ObjType::POINT3][ObjType::POINT3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointPointInteractor>(
                    static_cast<const Primitives::Point3&>(obj1),
                    static_cast<const Primitives::Point3&>(obj2)
                ); 
            };

        // =====================================================================

        table[ObjType::POINT3][ObjType::PLANE3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointPlaneInteractor>(
                    static_cast<const Primitives::Point3&>(obj1),
                    static_cast<const Primitives::Plane3&>(obj2)
                ); 
            };

        table[ObjType::PLANE3][ObjType::POINT3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointPlaneInteractor>(
                    static_cast<const Primitives::Point3&>(obj2),
                    static_cast<const Primitives::Plane3&>(obj1)
                ); 
            };

        // =====================================================================

        table[ObjType::POINT3][ObjType::LINE3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointLineInteractor>(
                    static_cast<const Primitives::Point3&>(obj1),
                     static_cast<const Primitives::Line3 &>(obj2)
                ); 
            };
        
        table[ObjType::LINE3][ObjType::POINT3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PointLineInteractor>(
                    static_cast<const Primitives::Point3&>(obj2),
                     static_cast<const Primitives::Line3 &>(obj1)
                ); 
            };

        // =====================================================================

        table[ObjType::LINE3][ObjType::LINE3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LineLineInteractor>(
                    static_cast<const Primitives::Line3&>(obj1),
                    static_cast<const Primitives::Line3&>(obj2)
                ); 
            };
        
        // =====================================================================

        table[ObjType::LINE3][ObjType::PLANE3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LinePlaneInteractor>(
                     static_cast<const Primitives::Line3 &>(obj1),
                    static_cast<const Primitives::Plane3&>(obj2)
                ); 
            };

        table[ObjType::PLANE3][ObjType::LINE3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<LinePlaneInteractor>(
                     static_cast<const Primitives::Line3 &>(obj2),
                    static_cast<const Primitives::Plane3&>(obj1)
                ); 
            };
        // =====================================================================

        table[ObjType::PLANE3][ObjType::PLANE3] = 
            [](const GeomObj& obj1, const GeomObj& obj2) {
                return std::make_unique<PlanePlaneInteractor>(
                    static_cast<const Primitives::Plane3&>(obj1),
                    static_cast<const Primitives::Plane3&>(obj2)
                ); 
            };
    
        return table;
    }();

    return interactor_table[obj1.WhoAmI()][obj2.WhoAmI()](obj1, obj2);
}

} // namespace Geometry/MathEngine