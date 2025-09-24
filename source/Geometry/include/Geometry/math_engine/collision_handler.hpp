#pragma once


#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"

#include <memory>


namespace Geometry::MathEngine {


enum CollisionCodeT
{  
    NOTHING  = 0,

    CROSS    = 1,
    OVERLAP  = 2,
    LIES_IN  = 3,
    EQUAL    = 4,
};

[[nodiscard]] std::string CollisionCodeStr(CollisionCodeT code);


class Interactor
{
public:
    [[nodiscard]] virtual const  CollisionCodeT CollisionCode() const = 0;
    [[nodiscard]] virtual        double         Distance     () const = 0;
    [[nodiscard]] virtual        GeomObjUniqPtr Intersect    () const = 0;

    [[nodiscard]] virtual const  GeomObj&       GeyObj1      () const = 0;
    [[nodiscard]] virtual const  GeomObj&       GeyObj2      () const = 0;
};

std::unique_ptr<Interactor> Interact(const GeomObj& obj1, const GeomObj& obj2);


class PointPointInteractor : public Interactor
{
public:
    PointPointInteractor(const Primitives::Point3& point1, const Primitives::Point3& point2)
        : point1_(point1)
        , point2_(point2)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return point1_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return point2_; };

private:
    const Primitives::Point3& point1_;
    const Primitives::Point3& point2_;
};


class PointPlaneInteractor : public Interactor
{
public:
    PointPlaneInteractor(const Primitives::Point3& point, const Primitives::Plane3& plane)
        : point_(point)
        , plane_(plane)
        {}

    PointPlaneInteractor(const Primitives::Plane3& plane, const Primitives::Point3& point)
        : point_(point)
        , plane_(plane)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return point_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return plane_; };

private:
    const Primitives::Point3& point_;
    const Primitives::Plane3& plane_;
};


class PointLineInteractor : public Interactor
{
public:
    PointLineInteractor(const Primitives::Point3& point, const Primitives::Line3& line)
        : point_(point)
        , line_ (line)
        {}

    PointLineInteractor(const Primitives::Line3& line, const Primitives::Point3& point)
        : point_(point)
        , line_ (line)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return point_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return line_ ; };

private:
    const Primitives::Point3& point_;
    const Primitives::Line3 & line_;
};



class LineLineInteractor : public Interactor
{
public:
    LineLineInteractor(const Primitives::Line3& line1, const Primitives::Line3& line2)
        : line1_(line1)
        , line2_(line2)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return line1_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return line2_; };

private:
    const Primitives::Line3& line1_;
    const Primitives::Line3& line2_;
};


class LinePlaneInteractor : public Interactor
{
public:
    LinePlaneInteractor(const Primitives::Line3& line, const Primitives::Plane3& plane)
        : line_ (line)
        , plane_(plane)
        {}

    LinePlaneInteractor(const Primitives::Plane3& plane, const Primitives::Line3& line) // TODO: удалить перегрузку конструкторов за ненадобностью
        : line_ (line)
        , plane_(plane)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return line_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return plane_; };

private:
    const Primitives::Line3 & line_;
    const Primitives::Plane3& plane_;
};


class PlanePlaneInteractor : public Interactor
{
public:
    PlanePlaneInteractor(const Primitives::Plane3& plane1, const Primitives::Plane3& plane2)
        : plane1_(plane1)
        , plane2_(plane2)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return plane1_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return plane2_; };

private:
    const Primitives::Plane3& plane1_;
    const Primitives::Plane3& plane2_;
};


class PointLinesectInteractor : public Interactor
{
public:
    PointLinesectInteractor( const Primitives::Point3& point, const Shapes::Linesect3& linesect)
        : point_   (point)
        , linesect_(linesect)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return point_;    };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return linesect_; };

private:
    const Primitives::Point3& point_;
    const Shapes::Linesect3 & linesect_;
};


class LineLinesectInteractor : public Interactor
{
public:
    LineLinesectInteractor(const Primitives::Line3& line, const Shapes::Linesect3& linesect)
        : line_    (line)
        , linesect_(linesect)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return line_;     };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return linesect_; };

private:
    const Primitives::Line3& line_;
    const Shapes::Linesect3& linesect_;
};


class LinesectLinesectInteractor : public Interactor
{
public:
    LinesectLinesectInteractor(const Shapes::Linesect3& linesect1, const Shapes::Linesect3& linesect2)
        : linesect1_(linesect1)
        , linesect2_(linesect2)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return linesect1_; };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return linesect2_; };

private:
    const Shapes::Linesect3& linesect1_;
    const Shapes::Linesect3& linesect2_;
};



class LineTriangleInteractor : public Interactor
{
public:
    LineTriangleInteractor(const Primitives::Line3& line, const Shapes::Triangle3& triangle)
        : line_    (line)
        , triangle_(triangle)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

    [[nodiscard]] virtual const GeomObj&       GeyObj1      () const override final { return line_;     };
    [[nodiscard]] virtual const GeomObj&       GeyObj2      () const override final { return triangle_; };

private:
    const Primitives::Line3& line_;
    const Shapes::Triangle3& triangle_;
};



}