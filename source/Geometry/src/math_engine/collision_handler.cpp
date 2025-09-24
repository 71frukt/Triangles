#include <cstddef>
#include <string>
#include <memory>

#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/point3.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/collision_handler.hpp"

#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"


namespace Geometry::MathEngine {

std::string CollisionCodeStr(CollisionCodeT code)
{
    switch (code) {
    
    case NOTHING: return "NOTHING";
    case CROSS  : return "CROSS"  ;
    case OVERLAP: return "OVERLAP";
    case LIES_IN: return "LIES_IN";
    case EQUAL  : return "EQUAL"  ;

    default     : return "UNKNOWN";

    }
}


// =========== COLLISION CODE ======================================================

const CollisionCodeT PointPointInteractor::CollisionCode() const
{
    if (point1_ == point2_)
        return EQUAL;

    else
        return NOTHING;
}

const CollisionCodeT PointPlaneInteractor::CollisionCode() const
{
    ASSERT_HANDLE(plane_.Assert());

    if (Math::DoubleZero( 
        plane_.GetA() * point_.GetX() + 
          plane_.GetB() * point_.GetY() + 
          plane_.GetC() * point_.GetZ() + plane_.GetD())
        )
    {
        return LIES_IN;
    }

    else
    {
        return NOTHING;
    }
}

const CollisionCodeT PointLineInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line_.Assert());

    Math::Vector3 r1 = line_.GetOrigin() - point_;

    if (Math::DoubleZero(Distance()))
        return LIES_IN;

    else
        return NOTHING;
}


const CollisionCodeT LineLineInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line1_.Assert());
    ASSERT_HANDLE(line2_.Assert());

    const Math::Vector3& normd_dir1 = line1_.GetNormdDir();
    const Math::Vector3& normd_dir2 = line2_.GetNormdDir();

    if (Math::DoubleZero(Distance()))
    {
        if (normd_dir1.Collinear(normd_dir2))
            return EQUAL;

        else
            return CROSS;
    }

    else
        return NOTHING;
}


const CollisionCodeT LinePlaneInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line_ .Assert());
    ASSERT_HANDLE(plane_.Assert());

    const Math::Vector3&      line_normd_dir   = line_.GetNormdDir();
    const Math::Vector3&      plane_normd_norm = plane_.GetNormdNormality();
    
    const Primitives::Point3& line_origin      = line_.GetOrigin();
    // const Primitives::Point3& plane_origin     = plane_.GetOrigin();

    if (line_normd_dir.Normal(plane_normd_norm))
    {
        if (Interact(line_origin, plane_)->CollisionCode() == LIES_IN)
            return LIES_IN;
        else
            return NOTHING;
    }

    else
        return CROSS;
}


const CollisionCodeT PlanePlaneInteractor::CollisionCode() const
{
    ASSERT_HANDLE(plane1_.Assert());
    ASSERT_HANDLE(plane2_.Assert());

    const Math::Vector3& plane1_normd_norm = plane1_.GetNormdNormality();
    const Math::Vector3& plane2_normd_norm = plane2_.GetNormdNormality();

    if (plane1_normd_norm.Collinear(plane2_normd_norm))
    {
        if (Math::DoubleEq(plane1_.GetD(), plane2_.GetD()))
            return EQUAL;

        else
            return NOTHING;;
    }

    else 
        return CROSS;
}


const CollisionCodeT PointLinesectInteractor::CollisionCode() const
{
    if (Interact(point_, linesect_.GetLine())->CollisionCode() == NOTHING)
        return NOTHING;

    // else if point lies on line
    // linesect [A, B], point C
    // if vector AC oppositely directed BC, point C lies on [A, B]

    Primitives::Point3 C = Interact(point_, linesect_.GetLine())->Intersect();

    Math::Vector3 AC = C - linesect_.GetPoint1();
    Math::Vector3 BC = C - linesect_.GetPoint2();
    Math::Vector3 AB = linesect_.GetPoint2() - linesect_.GetPoint1();


    if ((AC + BC).GetLen2() <= AB.GetLen2())
        return LIES_IN;
}


const CollisionCodeT LineLinesectInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line_    .Assert());
    ASSERT_HANDLE(linesect_.Assert());

    auto mb_cross_point = Interact(line_, linesect_.GetLine())->Intersect();

    return Interact(*mb_cross_point, linesect_)->CollisionCode();
}


const CollisionCodeT LinesectLinesectInteractor::CollisionCode() const
{
    ASSERT_HANDLE(linesect1_.Assert());
    ASSERT_HANDLE(linesect2_.Assert());

    auto cross_obj = Interact(linesect1_.GetLine(), linesect2_)->Intersect();

    if (cross_obj->WhoAmI() == POINT3) 
        return CROSS;

    else if (cross_obj->WhoAmI() == LINESECT3)
        return OVERLAP;

    else
        return NOTHING;

}


const CollisionCodeT LineTriangleInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line_    .Assert());
    ASSERT_HANDLE(triangle_.Assert());

    if (Interact(line_, triangle_.GetPlane())->CollisionCode() == LIES_IN)
        return CROSS;

    else
    {
        RLSU_WARNING("line is crossing triangle, We don't know how to count yet!");
        return NOTHING;
    }
}



// ==========/ COLLISION CODE ======================================================



// =========== DISTANCE ============================================================
double PointPointInteractor::Distance() const
{
    return (point2_ - point1_).GetLen();
}

double PointPlaneInteractor::Distance() const
{
    ASSERT_HANDLE(plane_.Assert());

    double numerator   = std::abs (plane_.GetA() * point_.GetX() +
                                      plane_.GetB() * point_.GetY() +
                                      plane_.GetC() * point_.GetZ() + plane_.GetD());

    double denominator = std::sqrt(plane_.GetA() * plane_.GetA() +
                                      plane_.GetB() * plane_.GetB() +
                                      plane_.GetC() * plane_.GetC());

    return numerator / denominator;
}

double PointLineInteractor::Distance() const
{
    ASSERT_HANDLE(line_.Assert());

    Math::Vector3 r1 = line_.GetOrigin() - point_;
    double hight = (r1 ^ line_.GetNormdDir()).GetLen();

    return hight;
}

double LineLineInteractor::Distance() const
{
    ASSERT_HANDLE(line1_.Assert());
    ASSERT_HANDLE(line2_.Assert());

    Math::Vector3 normd_h = line1_.GetNormdDir() ^ line2_.GetNormdDir();
    Math::Vector3 r1      = line2_.GetOrigin()   - line1_.GetOrigin();

    double hight = r1 * normd_h;

    return hight;
}


double LinePlaneInteractor::Distance() const
{
    ASSERT_HANDLE(line_ .Assert());
    ASSERT_HANDLE(plane_.Assert());

    if (CollisionCode() == NOTHING)
        return Interact(line_.GetOrigin(), plane_)->Distance();

    else
        return 0;
}


double PlanePlaneInteractor::Distance() const
{
    ASSERT_HANDLE(plane1_.Assert());
    ASSERT_HANDLE(plane2_.Assert());

    if (CollisionCode() == EQUAL)
        return 0;

    if (CollisionCode() == CROSS)
        return 0;

    // else if PARALLEL

    // (A, B, C) is normalized vector
    return std::abs(plane1_.GetD() - plane2_.GetD());
}


double PointLinesectInteractor::Distance() const
{
    return Interact(point_, linesect_.GetLine())->Distance();
}


double LineLinesectInteractor::Distance() const
{
    ASSERT_HANDLE(linesect_.Assert());
    ASSERT_HANDLE(line_    .Assert());

    return Interact(linesect_.GetLine(), line_)->Distance();
}


double LinesectLinesectInteractor::Distance() const
{
    ASSERT_HANDLE(linesect1_.Assert());
    ASSERT_HANDLE(linesect2_.Assert());

    return Interact(linesect1_.GetLine(), linesect2_.GetLine())->Distance();
}


double LineTriangleInteractor::Distance() const
{
    ASSERT_HANDLE(line_    .Assert());
    ASSERT_HANDLE(triangle_.Assert());

    RLSU_WARNING("Trying to get distance(line, trangle) We don't know how to count yet!");
    return 0;
}

// ==========/ DISTANCE ============================================================



// =========== INTERSECT ===========================================================

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

GeomObjUniqPtr PointLineInteractor::Intersect() const
{
    if (CollisionCode() == LIES_IN)
        return std::make_unique<Primitives::Point3>(point_);

    else
        return std::make_unique<NotAnObj>();
}

GeomObjUniqPtr LineLineInteractor::Intersect() const
{
    ASSERT_HANDLE(line1_.Assert());
    ASSERT_HANDLE(line2_.Assert());

    if (CollisionCode() == EQUAL)
        return std::make_unique<Primitives::Line3>(line1_);

    else if (CollisionCode() == CROSS)
    {
        const Math::Vector3& r1 = line1_.GetOrigin();
        const Math::Vector3& r2 = line2_.GetOrigin();
        const Math::Vector3& v1 = line1_.GetNormdDir();
        const Math::Vector3& v2 = line2_.GetNormdDir();

        const Math::Vector3& r0 = r2 - r1;

        // cross_point = r1 + t * v1
        // t = ([r0 ^ v1] * [v1 ^ v2]) / len2([v1 ^ v2])

        double t = ((r0 ^ v2) * (v1 ^ v2)) / (v1 ^ v2).GetLen2();

        Math::Vector3 rx = r1 + v1 * t;
        
        RLSU_ASSERT(Interact(Primitives::Point3(rx), line1_)->CollisionCode() == LIES_IN
                 && Interact(Primitives::Point3(rx), line2_)->CollisionCode() == LIES_IN);

        return std::make_unique<Primitives::Point3>(rx);
    }
    
    else
        return std::make_unique<NotAnObj>();
}


GeomObjUniqPtr LinePlaneInteractor::Intersect() const
{
    ASSERT_HANDLE(line_ .Assert());
    ASSERT_HANDLE(plane_.Assert());

    if (CollisionCode() == LIES_IN)
        return std::make_unique<Primitives::Line3>(line_);

    if (CollisionCode() == CROSS)
    {
        const Math::Vector3& r0_line = line_ .GetOrigin();
        const Math::Vector3& v_line  = line_ .GetNormdDir();
        const Math::Vector3& n_plane = plane_.GetNormdNormality();
        const double         D_plane = plane_.GetD();
        
        // cross_point = r0 + t * v
        // t = -((r0 * n) + D) / (v * n)
        
        double t = -((r0_line * n_plane) + D_plane) / (v_line * n_plane);
        
        Math::Vector3 rx = r0_line + v_line * t;
        
        RLSU_ASSERT(Interact(Primitives::Point3(rx), line_ )->CollisionCode() == LIES_IN
                 && Interact(Primitives::Point3(rx), plane_)->CollisionCode() == LIES_IN);

        return std::make_unique<Primitives::Point3>(rx);
    }

    else
        return std::make_unique<NotAnObj>();
}


GeomObjUniqPtr PlanePlaneInteractor::Intersect() const
{
    ASSERT_HANDLE(plane1_.Assert());
    ASSERT_HANDLE(plane2_.Assert());

    if (CollisionCode() == EQUAL)
        return std::make_unique<Primitives::Plane3>(plane1_);

    if (CollisionCode() == NOTHING)
        return std::make_unique<NotAnObj>();

    // else if CROSS
    // cross_line = r0 + v * t, v = [n1 ^ n2]
    // r0 = (D2 * [n1 ^ v] - D1 * [n2 ^ v]) / |v|**2

    const Math::Vector3& n1 = plane1_.GetNormdNormality();
    const Math::Vector3& n2 = plane2_.GetNormdNormality();
    const double         D1 = plane1_.GetD();
    const double         D2 = plane2_.GetD();

    const Math::Vector3  v  = n1 ^ n2;
    const Math::Vector3  r0 = ((n1 ^ v) * D2 - (n2 ^ v) * D1) / v.GetLen2();

    RLSU_ASSERT(Interact(Primitives::Line3(r0, v), plane1_)->CollisionCode() == LIES_IN
             && Interact(Primitives::Line3(r0, v), plane2_)->CollisionCode() == LIES_IN);

    return std::make_unique<Primitives::Line3>(r0, v);
}


GeomObjUniqPtr PointLinesectInteractor::Intersect() const
{
    ASSERT_HANDLE(linesect_.Assert());

    if (CollisionCode() == NOTHING)
        return std::make_unique<NotAnObj>();

    else
        return std::make_unique<Primitives::Point3>(point_);
}


GeomObjUniqPtr LineLinesectInteractor::Intersect() const
{
    ASSERT_HANDLE(line_    .Assert());
    ASSERT_HANDLE(linesect_.Assert());
    
    auto mb_cross_point = Interact(line_, linesect_.GetLine())->Intersect();

    return Interact(*mb_cross_point, linesect_)->Intersect();
}


GeomObjUniqPtr LinesectLinesectInteractor::Intersect() const
{
    ASSERT_HANDLE(linesect1_.Assert());
    ASSERT_HANDLE(linesect2_.Assert());

    auto cross_shape = Interact(linesect1_.GetLine(), linesect2_)->Intersect();

    if (cross_shape->WhoAmI() == POINT3)
        return Interact(*cross_shape, linesect1_)->Intersect();

    else if (cross_shape->WhoAmI() == LINE3)
    {
        GeomObjUniqPtr mb_cross_points[4] = 
        {
            Interact(linesect1_.GetPoint1(), linesect2_)->Intersect(),
            Interact(linesect1_.GetPoint2(), linesect2_)->Intersect(),
            Interact(linesect2_.GetPoint1(), linesect1_)->Intersect(),
            Interact(linesect2_.GetPoint2(), linesect1_)->Intersect(),
        };

        GeomObjUniqPtr cross_point1;
        GeomObjUniqPtr cross_point2;

        bool cross_point1_defined = false;
        bool cross_point2_defined = false;

        for (size_t i = 0; i < 4; i++)
        {
            if (mb_cross_points[i]->WhoAmI() == POINT3 && cross_point1_defined == false)
            {
                cross_point1_defined = true;
                cross_point1 = std::move(mb_cross_points[i]);
            }

            else if (mb_cross_points[i]->WhoAmI() == POINT3 && cross_point2_defined == false)
            {
                cross_point2_defined = true;
                cross_point2 = std::move(mb_cross_points[i]);
            }

            RLSU_ASSERT( !(mb_cross_points[i]->WhoAmI() == POINT3 && cross_point1_defined == false 
                                                                  && cross_point2_defined == false));
        }

        if (cross_point1->WhoAmI() == NOT_AN_OBJ && cross_point1->WhoAmI() == NOT_AN_OBJ)
            return std::make_unique<NotAnObj>();

        else
        {
            RLSU_ASSERT(cross_point1->WhoAmI() == POINT3 && cross_point2->WhoAmI() == POINT3);

            if (cross_point1 == cross_point2)
                return std::make_unique<Primitives::Point3>(cross_point1);

            else
                return std::make_unique<Shapes::Linesect3>(cross_point1, cross_point2);
        }
    }

    else
        return std::make_unique<NotAnObj>();

}


GeomObjUniqPtr LineTriangleInteractor::Intersect() const
{
    ASSERT_HANDLE(line_    .Assert());
    ASSERT_HANDLE(triangle_.Assert());

    // if (Interact(line_, triangle_.GetPlane())->CollisionCode() == LIES_IN)
    // {

    //     auto cross_point1 = Interact(line_, triangle_.GetLine1())->Intersect();
    //     auto cross_point2 = Interact(line_, triangle_.GetLine2())->Intersect();
    //     auto cross_point3 = Interact(line_, triangle_.GetLine3())->Intersect();

    //     if (cross_point1 == cross_point2 || cross_point1 == cross_point3 || cross_point2 == cross_point3)
    // }



    // else
    // {
    //     RLSU_WARNING("Trying to intersect crossing triangle and line, We don't know how to count yet!");
    //     return std::make_unique<NotAnObj>();
    // }
}


// ==========/ INTERSECT ===========================================================



}