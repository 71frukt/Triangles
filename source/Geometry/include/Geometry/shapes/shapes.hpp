#pragma once

#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"

namespace Geometry::Shapes {

class Shape : public GeomObj
{
public:
    virtual ~Shape() = default;

private:
};


class Linesect3 : public Shape
{
public:
    Linesect3(const Primitives::Point3& point1, const Primitives::Point3& point2)
        : point1_(point1)
        , point2_(point2)
        , line_  (point1, point2)
    { }

    virtual void Assert() const override;

    [[nodiscard]] const Primitives::Point3& GetPoint1() const { return point1_; }
    [[nodiscard]] const Primitives::Point3& GetPoint2() const { return point2_; }
    [[nodiscard]] const Primitives::Line3 & GetLine  () const { return line_  ; }

private:
    Primitives::Point3 point1_;
    Primitives::Point3 point2_;

    Primitives::Line3  line_;
};


class Triangle3 : public Shape
{
public:
    Triangle3(const Primitives::Point3& point1, const Primitives::Point3& point2, const Primitives::Point3& point3);
    Triangle3(const Primitives::Line3 & line1 , const Primitives::Line3 & line2 , const Primitives::Line3 & line3 );

    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::TRIANGLE3; };

    virtual void Assert() const override;
    virtual void Dump(const std::string& name = "some_obj") const override;

    [[nodiscard]] const Primitives::Plane3& GetPlane() const { return plane_; }

    [[nodiscard]] const Primitives::Line3&  GetLine1() const { return line1_; }
    [[nodiscard]] const Primitives::Line3&  GetLine2() const { return line2_; }
    [[nodiscard]] const Primitives::Line3&  GetLine3() const { return line3_; }

private:
    Primitives::Point3 point1_;
    Primitives::Point3 point2_;
    Primitives::Point3 point3_;

    Primitives::Line3  line1_;
    Primitives::Line3  line2_;
    Primitives::Line3  line3_;

    Primitives::Plane3 plane_;
};

} // namespace Geometry::Shapes