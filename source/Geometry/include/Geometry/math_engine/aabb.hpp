#pragma once

#include <cstddef>
#include <iostream>
#include <list>
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

namespace Geometry::MathEngine {


class AABBox
{
public:
    AABBox(const Primitives::Point3& p0, const Primitives::Point3& p1, const AABBox* father_ptr)
        : p0_(p0)
        , p1_(p1)
        , father(father_ptr)
    {}

    AABBox(const AABBox* father_ptr)
        : p0_()
        , p1_()
        , father(nullptr)
    {}

    AABBox() = default;

    virtual ~AABBox() = 0;

    [[nodiscard]] const Primitives::Point3& GetP0() const { return p0_; }
    [[nodiscard]] const Primitives::Point3& GetP1() const { return p1_; }

    const Primitives::Point3& SetP0(const Primitives::Point3& p0 )  { return (p0_ = p0); }
    const Primitives::Point3& SetP1(const Primitives::Point3& p1 )  { return (p1_ = p1); }

    double SetP0_X(double x)  { return p0_.SetX(x); }
    double SetP0_Y(double y)  { return p0_.SetY(y); }
    double SetP0_Z(double z)  { return p0_.SetZ(z); }
    
    double SetP1_X(double x)  { return p1_.SetX(x); }
    double SetP1_Y(double y)  { return p1_.SetY(y); }
    double SetP1_Z(double z)  { return p1_.SetZ(z); }

    const AABBox* father;
    
    virtual void Assert() const;

protected:
    Primitives::Point3 p0_;     // back  left  down
    Primitives::Point3 p1_;     // front right up
};


class AABLeaf : public AABBox
{
public:
    AABLeaf(const GeomObj* const inscribed, const AABBox* father_ptr);
    AABLeaf(const GeomObj* const inscribed) : AABLeaf(inscribed, nullptr) {}

    virtual void Assert() const override;

private:
    const GeomObj* const inscribed_;

    void BuildFromPoint_   ();
    void BuildFromLinesect_();
    void BuildFromTriangle_();
};


class AABContainer : public AABBox
{
public:
    AABContainer(const std::list<const AABBox*>& children_ref, const AABBox* father_ptr);
    AABContainer(const AABBox* father_ptr);
    AABContainer() = default;

    void UpdateSizeAccordToChildren();

    std::list<const AABBox*> children;
    
    virtual void Assert() const override;
private:
};


} // namespace Geometry::MathEngine