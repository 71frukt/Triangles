#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/common/geometry_obj.hpp"

#include "RLogSU/logger.hpp"

namespace Geometry::MathEngine {


const CollisionCodeT PointPointInteractor::CollisionCode() const
{
    if (point1_ == point2_)
        return EQUAL;

    else
        return ONE_TYPE;
}

const CollisionCodeT PointPlaneInteractor::CollisionCode() const
{
    if (plane_.GetA() * point_.GetX() + plane_.GetB() * point_.GetY() + plane_.GetC() * point_.GetZ() + plane_.GetD() == 0)
        return LIES_IN;

    else
        return DIFF_TYPE;
}


double PointPointInteractor::Distance() const
{
    return (point2_ - point1_).GetLen();
}

double PointPlaneInteractor::Distance() const
{
    ERROR_HANDLE(plane_.Verify());

    double numerator   = std::abs (plane_.GetA() * point_.GetX() + plane_.GetB() * point_.GetY() + plane_.GetC() * point_.GetZ() + plane_.GetD());
    double denominator = std::sqrt(plane_.GetA() * plane_.GetA() + plane_.GetB() * plane_.GetB() + plane_.GetC() * plane_.GetC());

    return numerator / denominator;
}


GeomObjUniqPtr PointPointInteractor::Intersect() const
{
    if (CollisionCode() == EQUAL)
        return std::make_unique<Primitives::Point3>(point1_);

    else
        return std::make_unique<NotAnObj>();
}

GeomObjUniqPtr PointPlaneInteractor::Intersect() const
{
    if (CollisionCode() == EQUAL)
        return std::make_unique<Primitives::Point3>(point_);

    else
        return std::make_unique<NotAnObj>();
}


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

        return table;
    }();


    return interactor_table[obj1.WhoAmI()][obj2.WhoAmI()](obj1, obj2);
    // interactor_table[ObjType::PLANE3][ObjType::PLANE3] = 
    //     [](GeomObj& obj1, GeomObj& obj2) {
    //         return std::make_unique<PlanePlaneInteractor>(
    //             static_cast<Primitives::Plane3&>(obj1),
    //             static_cast<Primitives::Plane3&>(obj2)
    //         ); 
    //     };
}

}