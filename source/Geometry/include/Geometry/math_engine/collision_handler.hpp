#pragma once


#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

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

    virtual ~Interactor() = default; 
};

std::unique_ptr<Interactor> Interact(const GeomObj& obj1, const GeomObj& obj2);


class NotImplementedInteractor : public Interactor
{
public:    
    NotImplementedInteractor(const GeomObj& some_obj1, const GeomObj& some_obj2) 
    {
        RLSU_ERROR("called not-iplemented interactor!");
    }

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final { RLSU_ERROR("called not-iplemented interactor!");return NOTHING; };
    
    [[nodiscard]] virtual       double         Distance     () const override final { return 0; }
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final { return std::make_unique<NotAnObj>(); };

private:
};


class NotAnObjInteractor : public Interactor
{
public:    
    NotAnObjInteractor(const GeomObj& notanobj, const GeomObj& some_obj) {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final { return NOTHING; };

    [[nodiscard]] virtual       double         Distance     () const override final 
    {
        RLSU_ERROR("Trying to get distance to not-an-obj"); 
        return 0;
    };

    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final { return std::make_unique<NotAnObj>(); };

private:
};


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

private:
    const Shapes::Linesect3& linesect1_;
    const Shapes::Linesect3& linesect2_;
};


class PointTriangleInteractor : public Interactor
{
public:
    PointTriangleInteractor(const Primitives::Point3& point, const Shapes::Triangle3& triangle)
        : point_   (point)
        , triangle_(triangle)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

private:
    const Primitives::Point3& point_;
    const Shapes::Triangle3 & triangle_;
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

private:
    const Primitives::Line3& line_;
    const Shapes::Triangle3& triangle_;
};


class LinesectTriangleInteractor : public Interactor
{
public:
    LinesectTriangleInteractor(const Shapes::Linesect3& linesect, const Shapes::Triangle3& triangle)
        : linesect_(linesect)
        , triangle_(triangle)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final;
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final;

private:
    const Shapes::Linesect3& linesect_;
    const Shapes::Triangle3& triangle_;
};


class TriangleTriangleInteractor : public Interactor
{
public:
    TriangleTriangleInteractor(const Shapes::Triangle3& triangle1, const Shapes::Triangle3& triangle2)
        : triangle1_(triangle1)
        , triangle2_(triangle2)
        {}

    [[nodiscard]] virtual const CollisionCodeT CollisionCode() const override final;
    [[nodiscard]] virtual       double         Distance     () const override final { return 0; };
    [[nodiscard]] virtual       GeomObjUniqPtr Intersect    () const override final { return std::make_unique<NotAnObj>(); };

private:
    const Shapes::Triangle3& triangle1_;
    const Shapes::Triangle3& triangle2_;

    [[nodiscard]] const CollisionCodeT EqualPlaneCollisionCode_() const;
    [[nodiscard]] const CollisionCodeT CrossPlaneCollisionCode_() const;
    [[nodiscard]] const CollisionCodeT CollisionCodeAlongLine_ (const Primitives::Line3& line) const;
};


}