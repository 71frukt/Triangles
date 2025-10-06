#pragma once

#include <cstddef>
#include <list>
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/primitives/primitives.hpp"

namespace Geometry::MathEngine {

class AABBox;    
class AABContainer;

using NodeList    = std::list<std::unique_ptr<const AABBox>>;
using NodeConstIt = NodeList::const_iterator;

enum NodeType
{
    AABBOX,
    CONTAINER,
    LEAF
};


class AABBox
{
friend class BvhTree;

public:
    AABBox(const Primitives::Point3& p0, const Primitives::Point3& p1, NodeConstIt father_it)
        : p0_(p0)
        , p1_(p1)
        , father(father_it)
    {}

    AABBox(NodeConstIt father_it)
        : p0_()
        , p1_()
        , father({})
    {}

    virtual ~AABBox() = default;

    NodeConstIt father;

    [[nodiscard]] virtual NodeType Type() const { return AABBOX; };

    [[nodiscard]] bool NodeItIsValid(NodeConstIt it) const;

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


class AABContainer : public AABBox
{
friend class BvhTree;

public:
    AABContainer(const std::list<NodeConstIt>& children_ref, NodeConstIt father_it);
    AABContainer(NodeConstIt father_it);

    [[nodiscard]] virtual NodeType Type() const override final { return CONTAINER; };

    [[nodiscard]] const std::list<NodeConstIt>& GetChildren   () const { return children_; }

    [[nodiscard]] size_t GetChildrenNum() const { return children_.size(); }
    [[nodiscard]] bool   IsEmpty       () const { return children_.size() == 0; }
    [[nodiscard]] bool   IsDegraded    () const { return children_.size() == 1; }
    [[nodiscard]] bool   ContainsChild (NodeConstIt child_it) const;
    [[nodiscard]] bool   NoVolume      () const;


    void UpdateSizeAccordChild (NodeConstIt child_it         );
    void AddChild              (NodeConstIt new_child_it     );
    void AbandonChild          (NodeConstIt unwanted_child_it);

    virtual void Assert() const override;

private:
    std::list<NodeConstIt> children_;

    bool first_child_added_ = false;
};


class AABLeaf : public AABBox
{
friend class BvhTree;

public:
    AABLeaf(const GeomObj& inscribed, NodeConstIt father_ptr);
    AABLeaf(const GeomObj& inscribed) : AABLeaf(inscribed, {}) {}

    [[nodiscard]] virtual NodeType Type() const override final { return LEAF; };

    virtual void Assert() const override;

private:
    const GeomObj& inscribed_;

    void BuildFromPoint_   ();
    void BuildFromLinesect_();
    void BuildFromTriangle_();
};

} // namespace Geometry::MathEngine