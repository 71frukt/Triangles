#include "Geometry/math/double_handle.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/error_handler.hpp"

#include <algorithm>

namespace Geometry::MathEngine {

    
AABContainer::AABContainer(const std::list<const AABBox*>& children_ref, const AABBox* father_ptr)
    : AABBox(father_ptr)
    , children(children_ref)
{
    UpdateSizeAccordToChildren();
}


AABContainer::AABContainer(const AABBox* father_ptr)
    : AABBox(father_ptr)
    , children()
{}

void AABContainer::UpdateSizeAccordToChildren()
{
    double min_x = 0, max_x = 0;
    double min_y = 0, max_y = 0;
    double min_z = 0, max_z = 0;

    for (const AABBox* child : children)
    {
        ASSERT_HANDLE(child->Assert());

        if (child->GetP0().GetX() < min_x)  min_x = child->GetP0().GetX();
        if (child->GetP0().GetY() < min_y)  min_y = child->GetP0().GetY();
        if (child->GetP0().GetZ() < min_z)  min_z = child->GetP0().GetZ();
        
        if (child->GetP1().GetX() > max_x)  max_x = child->GetP1().GetX();
        if (child->GetP1().GetY() > max_y)  max_y = child->GetP1().GetY();
        if (child->GetP1().GetZ() > max_z)  max_z = child->GetP1().GetZ();
    }

    p0_ = {min_x - Math::CmpEps, min_y - Math::CmpEps, min_z - Math::CmpEps};
    p1_ = {max_x + Math::CmpEps, max_y + Math::CmpEps, max_z + Math::CmpEps};
}

AABLeaf::AABLeaf(const GeomObj* const inscribed, const AABBox* father_ptr)
    : AABBox(father_ptr)
    , inscribed_(inscribed)
{
    ASSERT_HANDLE(inscribed->Assert());

    switch (inscribed_->WhoAmI()) {
        case POINT3    : BuildFromPoint_   ();  break;
        case LINESECT3 : BuildFromLinesect_();  break;
        case TRIANGLE3 : BuildFromTriangle_();  break;
        
        default        : RLSU_ERROR("invalid owner type = '{}'", ObjTypeStr(inscribed_->WhoAmI()));        
    }
}


void AABLeaf::BuildFromPoint_()
{
    RLSU_ASSERT(inscribed_->WhoAmI() == POINT3);

    auto inscribed_point = static_cast<const Geometry::Primitives::Point3* const>(inscribed_);

    p0_.SetX(inscribed_point->GetX());
    p0_.SetY(inscribed_point->GetY());
    p0_.SetZ(inscribed_point->GetZ());

    p0_.SetX(inscribed_point->GetX());
    p0_.SetY(inscribed_point->GetY());
    p0_.SetZ(inscribed_point->GetZ());
}


void AABLeaf::BuildFromLinesect_()
{
    RLSU_ASSERT(inscribed_->WhoAmI() == LINESECT3);

    auto inscribed_linesect = static_cast<const Geometry::Shapes::Linesect3* const>(inscribed_);
    
    double x0_ = std::min(inscribed_linesect->GetPoint1().GetX(), inscribed_linesect->GetPoint2().GetX());
    double x1_ = std::max(inscribed_linesect->GetPoint1().GetX(), inscribed_linesect->GetPoint2().GetX());

    double y0_ = std::min(inscribed_linesect->GetPoint1().GetY(), inscribed_linesect->GetPoint2().GetY());
    double y1_ = std::max(inscribed_linesect->GetPoint1().GetY(), inscribed_linesect->GetPoint2().GetY());

    double z0_ = std::min(inscribed_linesect->GetPoint1().GetZ(), inscribed_linesect->GetPoint2().GetZ());
    double z1_ = std::max(inscribed_linesect->GetPoint1().GetZ(), inscribed_linesect->GetPoint2().GetZ());

    p0_.SetX(x0_);
    p0_.SetY(x0_);
    p0_.SetZ(x0_);

    p1_.SetX(x1_);
    p1_.SetY(x1_);
    p1_.SetZ(x1_);
}


void AABLeaf::BuildFromTriangle_()
{
    RLSU_ASSERT(inscribed_->WhoAmI() == TRIANGLE3);
    
    auto inscribed_triangle = static_cast<const Geometry::Shapes::Triangle3* const>(inscribed_);

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


void AABBox::Assert() const
{
    RLSU_ASSERT(father);
    RLSU_ASSERT(p0_.GetX() <= p1_.GetX());
    RLSU_ASSERT(p0_.GetY() <= p1_.GetY());
    RLSU_ASSERT(p0_.GetZ() <= p1_.GetZ());
}

void AABLeaf::Assert() const
{
    AABBox::Assert();
    inscribed_->Assert();
}

void AABContainer::Assert() const
{
    AABBox::Assert();

    for (auto child : children)
    {
        // checking that the shape is at least PARTIALLY in the box

        RLSU_ASSERT(child->GetP1().GetX() >= p0_.GetX());
        RLSU_ASSERT(child->GetP1().GetY() >= p0_.GetY());
        RLSU_ASSERT(child->GetP1().GetZ() >= p0_.GetX());
        
        RLSU_ASSERT(child->GetP0().GetX() <= p1_.GetX());
        RLSU_ASSERT(child->GetP0().GetY() <= p1_.GetY());
        RLSU_ASSERT(child->GetP0().GetZ() <= p1_.GetX());
    }
}

}