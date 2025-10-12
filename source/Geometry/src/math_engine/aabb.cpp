#include "Geometry/math/double_handle.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

#include <algorithm>

namespace Geometry::MathEngine {

Primitives::Point3 AABBox::MaxAxisPoint(const Primitives::Point3 &a, const Primitives::Point3 &b)
{
    double max_x = std::max(a.GetX(), b.GetX());
    double max_y = std::max(a.GetY(), b.GetY());
    double max_z = std::max(a.GetZ(), b.GetZ());

    return {max_x, max_y, max_z};
}

Primitives::Point3 AABBox::MinAxisPoint(const Primitives::Point3 &a, const Primitives::Point3 &b)
{
    double min_x = std::min(a.GetX(), b.GetX());
    double min_y = std::min(a.GetY(), b.GetY());
    double min_z = std::min(a.GetZ(), b.GetZ());

    return {min_x, min_y, min_z};
}


AABContainer::AABContainer(const std::list<AABBox*>& children_ref, const AABBox* father_ptr)
    : AABBox(father_ptr)
{
    if (children_ref.size() == 0)
    {
        p0_ = p1_ = {0, 0, 0};
        return;
    }

    for (AABBox* const adding_child : children_ref)
    {
        AddChild(adding_child);
    }
}


AABContainer::AABContainer(const AABContainer* father_ptr)
    : AABBox(father_ptr)
    , children_()
{}


void AABContainer::MoveChildFromOtherContainer(const std::list<AABBox*>::const_iterator& child_it, AABContainer* const other_cont)
{
    AABBox* child = *child_it;

    this->children_.splice(this->children_.end(), other_cont->children_, child_it);

    child->father = this;

    UpdateSizeAccordChild(child);
}

void AABContainer::AddChild(AABBox* new_child)
{
    RLSU_ASSERT(new_child);

    children_.push_back(new_child);
    new_child->father = this;

    UpdateSizeAccordChild(new_child);
}

void AABContainer::AbandonChild(AABBox* unwanted_child)
{
    RLSU_ASSERT(unwanted_child);
    RLSU_ASSERT(ContainsChild(unwanted_child));

    std::list<AABBox*>::iterator unwanted_it = std::find(children_.begin(), children_.end(), unwanted_child);
    children_.erase(unwanted_it);
}


void AABContainer::UpdateSizeAccordChild(const AABBox* child)
{

    if (!first_child_added_)
    {
        p0_ = child->GetP0();
        p1_ = child->GetP1();
        first_child_added_ = true;
    }

    RLSU_ASSERT(ContainsChild(child));

    this->SetP0(MinAxisPoint(this->GetP0(), child->GetP0()));
    this->SetP1(MaxAxisPoint(this->GetP1(), child->GetP1()));
}

bool AABContainer::ContainsChild(const AABBox* child) const
{
    return (std::find(children_.begin(), children_.end(), child) != children_.end());
}


AABLeaf::AABLeaf(const GeomObj* const inscribed, const AABBox* father_ptr, int leaf_id)
    : AABBox(father_ptr)
    , inscribed(inscribed)
    , id(leaf_id)
{
    ASSERT_HANDLE(inscribed->Assert());

    switch (inscribed->WhoAmI()) {
        case POINT3    : ERROR_HANDLE(BuildFromPoint_   ());  break;
        case LINESECT3 : ERROR_HANDLE(BuildFromLinesect_());  break;
        case TRIANGLE3 : ERROR_HANDLE(BuildFromTriangle_());  break;
        
        default        : RLSU_ERROR("invalid owner type = '{}'", ObjTypeStr(inscribed->WhoAmI()));        
    }
}


void AABLeaf::BuildFromPoint_()
{
    RLSU_ASSERT(inscribed->WhoAmI() == POINT3);

    auto inscribed_point = static_cast<const Geometry::Primitives::Point3* const>(inscribed);

    p0_.SetX(inscribed_point->GetX() - Math::CmpEps);
    p0_.SetY(inscribed_point->GetY() - Math::CmpEps);
    p0_.SetZ(inscribed_point->GetZ() - Math::CmpEps);

    p1_.SetX(inscribed_point->GetX() + Math::CmpEps);
    p1_.SetY(inscribed_point->GetY() + Math::CmpEps);
    p1_.SetZ(inscribed_point->GetZ() + Math::CmpEps);
}


void AABLeaf::BuildFromLinesect_()
{
    RLSU_WARNING("building from linesect~~!!");

    RLSU_ASSERT(inscribed->WhoAmI() == LINESECT3);

    auto inscribed_linesect = static_cast<const Geometry::Shapes::Linesect3* const>(inscribed);
    
    double x0_ = std::min(inscribed_linesect->GetPoint1().GetX(), inscribed_linesect->GetPoint2().GetX());
    double x1_ = std::max(inscribed_linesect->GetPoint1().GetX(), inscribed_linesect->GetPoint2().GetX());

    double y0_ = std::min(inscribed_linesect->GetPoint1().GetY(), inscribed_linesect->GetPoint2().GetY());
    double y1_ = std::max(inscribed_linesect->GetPoint1().GetY(), inscribed_linesect->GetPoint2().GetY());

    double z0_ = std::min(inscribed_linesect->GetPoint1().GetZ(), inscribed_linesect->GetPoint2().GetZ());
    double z1_ = std::max(inscribed_linesect->GetPoint1().GetZ(), inscribed_linesect->GetPoint2().GetZ());

    p0_.SetX(x0_);
    p0_.SetY(y0_);
    p0_.SetZ(z0_);

    p1_.SetX(x1_);
    p1_.SetY(y1_);
    p1_.SetZ(z1_);
}


void AABLeaf::BuildFromTriangle_()
{
    RLSU_ASSERT(inscribed->WhoAmI() == TRIANGLE3);
    
    auto inscribed_triangle = static_cast<const Geometry::Shapes::Triangle3* const>(inscribed);

    double tr_x1 = inscribed_triangle->GetPoint1().GetX();
    double tr_x2 = inscribed_triangle->GetPoint2().GetX();
    double tr_x3 = inscribed_triangle->GetPoint3().GetX();

    double tr_y1 = inscribed_triangle->GetPoint1().GetY();
    double tr_y2 = inscribed_triangle->GetPoint2().GetY();
    double tr_y3 = inscribed_triangle->GetPoint3().GetY();

    double tr_z1 = inscribed_triangle->GetPoint1().GetZ();
    double tr_z2 = inscribed_triangle->GetPoint2().GetZ();
    double tr_z3 = inscribed_triangle->GetPoint3().GetZ();


    double max_x = std::max({tr_x1, tr_x2, tr_x3});
    double min_x = std::min({tr_x1, tr_x2, tr_x3});

    double max_y = std::max({tr_y1, tr_y2, tr_y3});
    double min_y = std::min({tr_y1, tr_y2, tr_y3});

    double max_z = std::max({tr_z1, tr_z2, tr_z3});
    double min_z = std::min({tr_z1, tr_z2, tr_z3});

    p0_.SetX(min_x);
    p1_.SetX(max_x);

    p0_.SetY(min_y);
    p1_.SetY(max_y);

    p0_.SetZ(min_z);
    p1_.SetZ(max_z);
}


}