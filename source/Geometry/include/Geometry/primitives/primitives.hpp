#pragma once

#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/point3.hpp"

#include "RLogSU/logger.hpp"

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
    Point3(double x, double y, double z) : Geometry::Math::Point3(x, y, z) {}

    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::POINT3; };

    virtual void Dump(const std::string& name = "some_obj") const override;

private:

};


class Line3 : public Primitive
{
public:
    Line3(Point3 origin, Math::Vector3 director) : origin_(origin), normalized_director_(director.Normalized()) {}

    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::LINE3; };
    
    virtual void Dump(const std::string& name = "some_obj") const override;

private:
    Point3 origin_;
    Math::Vector3 normalized_director_;
};


class Plane3 : public Primitive
{
public:

    Plane3(const Math::Vector3& normality_, const double D) : normd_normality_(normality_.Normalized()), D_(D) {};
    Plane3(const double A, const double B, const double C, const double D);
    Plane3(Point3 A, Point3 B, Point3 C);

    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::PLANE3; };

    [[nodiscard]] Math::Vector3 GetNormdNormality() const { return normd_normality_; };

    [[nodiscard]] double GetA() const { return normd_normality_.GetX(); };
    [[nodiscard]] double GetB() const { return normd_normality_.GetY(); };
    [[nodiscard]] double GetC() const { return normd_normality_.GetZ(); };
    [[nodiscard]] double GetD() const { return D_;                      };
    

    void Verify() const
    {
        RLSU_VERIFY(normd_normality_.GetLen2() != 0);
    }

    virtual void Dump(const std::string& name = "some_obj") const override;

private:

    Math::Vector3 normd_normality_;
    double  D_;
};


}