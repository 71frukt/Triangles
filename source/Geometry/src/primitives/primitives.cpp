#include "Geometry/primitives/primitives.hpp"
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/collision_handler.hpp"

#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

namespace Geometry::Primitives {

int Point3::CompareX(const Point3 &point1, const Point3 &point2)
{
    if (Math::DoubleG(point1.GetX(), point2.GetX()))
        return 1;

    else if (Math::DoubleB(point1.GetX(), point2.GetX()))
        return -1;

    else
        return 0;
}

int Point3::CompareY(const Point3 &point1, const Point3 &point2)
{
    if (Math::DoubleG(point1.GetY(), point2.GetY()))
        return 1;

    else if (Math::DoubleB(point1.GetY(), point2.GetY()))
        return -1;

    else
        return 0;
}

int Point3::CompareZ(const Point3 &point1, const Point3 &point2)
{
    if (Math::DoubleG(point1.GetZ(), point2.GetZ()))
        return 1;

    else if (Math::DoubleB(point1.GetZ(), point2.GetZ()))
        return -1;

    else
        return 0;
}


Math::Point3 Point3::CastFromGeomObj_(const GeomObjUniqPtr& game_obj)
{
    RLSU_ASSERT(game_obj->WhoAmI() == POINT3);
    auto point_ptr = dynamic_cast<const Geometry::Math::Point3*>(game_obj.get());
    RLSU_ASSERT(point_ptr);

    return *point_ptr;

    ERROR_HANDLE(Assert());
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

    ERROR_HANDLE(Assert());
}

Plane3::Plane3(const double A, const double B, const double C, const double D)
{
    Math::Vector3 normality(A, B, C);

    normd_normality_ = normality.Normalized();
    positive_D_ = D / normality.GetLen();

    if (!Math::DoublePositive(D))
    {
        positive_D_      *= -1;
        normd_normality_ *= -1;
    }

    ERROR_HANDLE(Assert());
}

Plane3::Plane3(const Point3& A, const Point3& B, const Point3& C)
{
    Math::Vector3 ab = B - A;
    Math::Vector3 ac = B - C;
    
    normd_normality_ = (ab ^ ac).Normalized();

    Math::Vector3 rad_vec_0(A);
    positive_D_ = -(rad_vec_0 * normd_normality_);

    if (!Math::DoublePositive(positive_D_))
    {
        positive_D_      *= -1;
        normd_normality_ *= -1;
    }

    RLSU_ASSERT(MathEngine::Interact(A, *this)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(B, *this)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(C, *this)->CollisionCode() == MathEngine::LIES_IN);

    ERROR_HANDLE(Assert());
}

bool Plane3::operator== (const Plane3& other ) const
{
    if (Math::DoubleZero(this->GetD()) && Math::DoubleZero(other.GetD()))
        return (this->normd_normality_.Collinear(other.normd_normality_));

    else
        return (this->normd_normality_ == other.normd_normality_
             && Math::DoubleEq(other.GetD(), this->GetD()));
}


}