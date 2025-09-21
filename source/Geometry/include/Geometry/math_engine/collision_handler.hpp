#pragma once


#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include <memory>
#include <functional>


namespace Geometry::MathEngine {


enum CollisionCodeT
{  
    DIFF_TYPE    = 0 << 0,
    ONE_TYPE     = 1 << 0,

    NOT_CROSS    = 0 << 1,
    CROSS        = 1 << 1,

    NOT_PARALLEL = 0 << 2,
    PARALLEL     = 1 << 2,

    LIES_IN      = CROSS   && PARALLEL,
    EQUAL        = LIES_IN && ONE_TYPE
};


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


// class PlanePlaneInteractor : public Interactor
// {
// public:
//     PlanePlaneInteractor(const Primitives::Plane3& plane1, const Primitives::Plane3& plane2)
//         : plane1_(plane1)
//         , plane2_(plane2)
//         {}

//     [[nodiscard]] virtual double            Distance () const override final;
//     [[nodiscard]] virtual GeomObj       Intersect() const override final;

//     [[nodiscard]] virtual const GeomObj& GeyObj1 () const override final { return plane1_; };
//     [[nodiscard]] virtual const GeomObj& GeyObj2 () const override final { return plane2_; };

// private:
//     const Primitives::Plane3& plane1_;
//     const Primitives::Plane3& plane2_;
// };

// GeomObj CollisionPlanePlane(const Primitives::Plane3& plane1, const Primitives::Plane3& plane2);



}