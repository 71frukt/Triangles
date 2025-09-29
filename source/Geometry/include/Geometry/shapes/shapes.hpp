#pragma once

#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "RLogSU/error_handler.hpp"

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
        , line_  (ERROR_HANDLE(Primitives::Line3(point1, point2)))
    { 
        RLSU_VERIFY(point1 != point2);
    }

    Linesect3() = default;

    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::LINESECT3; };

    virtual void Assert() const override;

    [[nodiscard]] const Primitives::Point3& GetPoint1() const { return point1_; }
    [[nodiscard]] const Primitives::Point3& GetPoint2() const { return point2_; }
    [[nodiscard]] const Primitives::Line3 & GetLine  () const { return line_  ; }

    [[nodiscard]] bool  operator== (const Linesect3& other) const;

private:
    Primitives::Point3 point1_;
    Primitives::Point3 point2_;

    Primitives::Line3  line_;

protected:
    virtual void DumpDetails() const override;
};


class Triangle3 : public Shape
{
public:
    Triangle3(const Primitives::Point3& point1, const Primitives::Point3& point2, const Primitives::Point3& point3);
    Triangle3(const Primitives::Line3 & line1 , const Primitives::Line3 & line2 , const Primitives::Line3 & line3 );

    [[nodiscard]] static GeomObjUniqPtr BuildGeomObj(const Primitives::Point3& point1, const Primitives::Point3& point2, const Primitives::Point3& point3);

    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::TRIANGLE3; };

    virtual void Assert() const override;

    [[nodiscard]] const Primitives::Plane3& GetPlane() const { return plane_; }

    [[nodiscard]] const Primitives::Point3& GetPoint1() const { return point1_; }
    [[nodiscard]] const Primitives::Point3& GetPoint2() const { return point2_; }
    [[nodiscard]] const Primitives::Point3& GetPoint3() const { return point3_; }

    [[nodiscard]] const          Linesect3& GetSide1 () const { return side1_; }
    [[nodiscard]] const          Linesect3& GetSide2 () const { return side2_; }
    [[nodiscard]] const          Linesect3& GetSide3 () const { return side3_; }

private:
    Primitives::Point3 point1_;
    Primitives::Point3 point2_;
    Primitives::Point3 point3_;

    Linesect3          side1_;
    Linesect3          side2_;
    Linesect3          side3_;

    Primitives::Plane3 plane_;

protected:
    virtual void DumpDetails() const override;
};

} // namespace Geometry::Shapes