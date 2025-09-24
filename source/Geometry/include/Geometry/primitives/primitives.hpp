#pragma once

#include "Geometry/common/geometry_obj.hpp"

#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/math.hpp"
#include "RLogSU/error_handler.hpp"

namespace Geometry::Primitives {

class Primitive : public GeomObj
{
public:
    virtual ~Primitive() = default;

private:
};


class Point3 : public Geometry::Math::Point3
             , public Primitive
{
public:
    Point3(double x, double y, double z)   : Geometry::Math::Point3(x, y, z)                    {}
    Point3(const Math::Vector3& rad_vec)   : Geometry::Math::Point3(rad_vec)                    {}
    Point3(const GeomObjUniqPtr& geom_obj) : Geometry::Math::Point3(CastFromGeomObj_(geom_obj)) {}

    [[nodiscard]] virtual ObjType WhoAmI() const override { return POINT3; };
    
    virtual void Assert() const override { };
    virtual void Dump(const std::string& name = "some_obj") const override;

private:
    Geometry::Math::Point3 CastFromGeomObj_(const GeomObjUniqPtr& game_obj);
};


class Line3 : public Primitive
{
public:
    Line3(const Point3& origin, const Math::Vector3& director) : origin_(origin), normd_dir_(director         .Normalized()) {}
    Line3(const Point3& point1, const Point3&        point2)   : origin_(point1), normd_dir_((point2 - point1).Normalized()) {}
    Line3(const GeomObjUniqPtr& geom_obj)                      : Line3(CastFromGeomObj_(geom_obj))                  {}

    [[nodiscard]] virtual ObjType WhoAmI() const override { return ObjType::LINE3; };
    
    virtual void Dump(const std::string& name = "some_obj") const override;

    virtual void Assert() const override
    {
        RLSU_ASSERT(Math::DoubleEq(normd_dir_.GetLen2(), 1), "actual length = {}", normd_dir_.GetLen());
    }

    [[nodiscard]] const Point3&        GetOrigin  () const { return origin_; } 
    [[nodiscard]] const Math::Vector3& GetNormdDir() const { return normd_dir_; } 

private:
    Point3 origin_;
    Math::Vector3 normd_dir_;

    Geometry::Primitives::Line3 CastFromGeomObj_(const GeomObjUniqPtr& game_obj);
};


class Plane3 : public Primitive
{
public:

    Plane3(const Math::Vector3& normality_, const double D);
    Plane3(const double A, const double B, const double C, const double D);
    Plane3(const Point3& A, const Point3& B, const Point3& C);
    Plane3(const GeomObjUniqPtr& geom_obj) : Plane3(CastFromGeomObj_(geom_obj)) {}

    [[nodiscard]] virtual ObjType WhoAmI() const override { return ObjType::PLANE3; };

    [[nodiscard]] Math::Vector3 GetNormdNormality() const { return normd_normality_; };

    [[nodiscard]] double GetA() const { return normd_normality_.GetX(); };
    [[nodiscard]] double GetB() const { return normd_normality_.GetY(); };
    [[nodiscard]] double GetC() const { return normd_normality_.GetZ(); };
    [[nodiscard]] double GetD() const { return positive_D_;             };
    

    void Assert() const override
    {
        RLSU_ASSERT(Math::DoubleEq(normd_normality_.GetLen2(), 1));
        RLSU_ASSERT(Math::DoublePositive(positive_D_));
    }

    virtual void Dump(const std::string& name = "some_obj") const override;

private:
    Math::Vector3 normd_normality_;
    double  positive_D_;

    Geometry::Primitives::Plane3 CastFromGeomObj_(const GeomObjUniqPtr& game_obj);
};


}