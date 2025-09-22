#include <string>
#include <memory>

#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/math/math.hpp"

#include "RLogSU/logger.hpp"


namespace Geometry::MathEngine {

std::string CollisionCodeStr(CollisionCodeT code)
{
    std::string code_str;

    if (code == EQUAL)     return      "EQUAL ";
    if (code == LIES_IN)   return      "LIES_IN ";
    if (code == COLLINEAR) return      "COLLINEAR "; 
    if (code == NOTHING)   return      "NOTHING "; 
    if (code == SKEW)      return      "SKEW";

    if (code & CROSS)      code_str += "CROSS ";

    if (code & PARALLEL)   code_str += "PARALLEL ";

    return code_str;
}


// =========== COLLISION CODE ======================================================

const CollisionCodeT PointPointInteractor::CollisionCode() const
{
    if (point1_ == point2_)
        return EQUAL;

    else
        return COLLINEAR;
}

const CollisionCodeT PointPlaneInteractor::CollisionCode() const
{
    ASSERT_HANDLE(plane_.Assert());

    if (plane_.GetA() * point_.GetX() + 
        plane_.GetB() * point_.GetY() + 
        plane_.GetC() * point_.GetZ() + plane_.GetD() == 0)
    {
        return LIES_IN;
    }

    else
        return NOTHING;
}

const CollisionCodeT PointLineInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line_.Assert());

    Math::Vector3 r1 = line_.GetOrigin() - point_;

    if (Distance() == 0)
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

    if (Distance() == 0)
    {
        if (normd_dir1.Collinear(normd_dir2))
            return EQUAL;

        else
            return CROSS;
    }

    else
        return SKEW;
}


const CollisionCodeT LinePlaneInteractor::CollisionCode() const
{
    ASSERT_HANDLE(line_ .Assert());
    ASSERT_HANDLE(plane_.Assert());

    const Math::Vector3&      line_normd_dir   = line_.GetNormdDir();
    const Math::Vector3&      plane_normd_norm = plane_.GetNormdNormality();
    
    const Primitives::Point3& line_origin      = line_.GetOrigin();
    // const Primitives::Point3& plane_origin     = plane_.GetOrigin();

    int code = NOTHING;

    if (Interact(line_origin, plane_)->CollisionCode() == LIES_IN)
        code |= CROSS;
    
    if (line_normd_dir.Normal(plane_normd_norm))
        code |= PARALLEL;

    else
        code |= CROSS;

    return CollisionCodeT(code);
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

    if (CollisionCode() == PARALLEL)
        return Interact(line_.GetOrigin(), plane_)->Distance();

    else
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

// ==========/ INTERSECT ===========================================================



// =========== INTERACT ============================================================

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

        return table;
    }();

    return interactor_table[obj1.WhoAmI()][obj2.WhoAmI()](obj1, obj2);
}

// ==========/ INTERACT ============================================================


}