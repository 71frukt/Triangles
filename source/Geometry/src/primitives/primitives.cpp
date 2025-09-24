#include "Geometry/primitives/primitives.hpp"
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/collision_handler.hpp"

#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

namespace Geometry::Primitives {

Geometry::Math::Point3 Point3::CastFromGeomObj_(const GeomObjUniqPtr& game_obj)
{
    RLSU_ASSERT(game_obj->WhoAmI() == POINT3);
    auto point_ptr = dynamic_cast<const Geometry::Math::Point3*>(game_obj.get());
    RLSU_ASSERT(point_ptr);

    return *point_ptr;
}


Line3 Line3::CastFromGeomObj_(const GeomObjUniqPtr& game_obj)
{
    RLSU_ASSERT(game_obj->WhoAmI() == LINE3);
    auto point_ptr = dynamic_cast<const Line3*>(game_obj.get());
    RLSU_ASSERT(point_ptr);

    return *point_ptr;
}


Plane3 Plane3::CastFromGeomObj_(const GeomObjUniqPtr& game_obj)
{
    RLSU_ASSERT(game_obj->WhoAmI() == PLANE3);
    auto point_ptr = dynamic_cast<const Plane3*>(game_obj.get());
    RLSU_ASSERT(point_ptr);

    return *point_ptr;
}

Plane3::Plane3(const Math::Vector3& normality_, const double D)
    : normd_normality_(normality_.Normalized())
    , positive_D_(D)
{
    if (!Math::DoublePositive(D))
    {
        positive_D_      *= -1;
        normd_normality_ *= -1;
    }
}

Plane3::Plane3(const double A, const double B, const double C, const double D)
{
    Math::Vector3 normality(A, B, C);

    normd_normality_ = normality.Normalized();
    positive_D_ = D / normality.GetLen();
}

Plane3::Plane3(const Point3& A, const Point3& B, const Point3& C)
{
    Math::Vector3 ab = B - A;
    Math::Vector3 ac = B - C;
    
    normd_normality_ = (ab ^ ac).Normalized();

    Math::Vector3 rad_vec_0(A);
    positive_D_ = -(rad_vec_0 * normd_normality_);

    RLSU_ASSERT(MathEngine::Interact(A, *this)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(B, *this)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(C, *this)->CollisionCode() == MathEngine::LIES_IN);
}

}