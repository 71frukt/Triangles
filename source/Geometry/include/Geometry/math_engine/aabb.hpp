#pragma once

#include <cstddef>
#include <list>
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"

namespace Geometry::MathEngine {

class AABContainer;

class AABBox
{
public:
    AABBox(const Primitives::Point3& p0, const Primitives::Point3& p1, AABContainer* father_ptr)
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
    virtual ~AABBox() = default;

    AABContainer* father;

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

    [[nodiscard]] static Primitives::Point3 MaxAxisPoint(const Primitives::Point3& a, const Primitives::Point3& b);
    [[nodiscard]] static Primitives::Point3 MinAxisPoint(const Primitives::Point3& a, const Primitives::Point3& b);

    virtual void Assert() const;
            void Dump  () const;

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
    AABContainer(const std::list<AABBox*>& children_ref, const AABBox* father_ptr);
    AABContainer(const AABContainer* father_ptr);
    AABContainer() = default;

    [[nodiscard]] const std::list<AABBox*>& GetChildren   () const { return children_; }
    [[nodiscard]]       size_t              GetChildrenNum() const { return children_.size(); }
    [[nodiscard]]       bool                IsEmpty       () const { return children_.size() == 0; }
    [[nodiscard]]       bool                IsDegraded    () const { return children_.size() == 1; }
    [[nodiscard]]       bool                ContainsChild (const AABBox& child) const;

    void MoveChildFromOtherContainer(const std::list<AABBox*>::const_iterator& child_it, AABContainer* const other_cont);

    void UpdateSizeAccordChild (const AABBox& child         );
    void AddChild              (      AABBox& new_child     );
    void AbandonChild          (      AABBox& unwanted_child);

    virtual void Assert() const override;

private:
    std::list<AABBox*> children_;

    bool first_child_added_ = false;
};


} // namespace Geometry::MathEngine