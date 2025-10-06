#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/vector3.hpp"
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


AABContainer::AABContainer(const std::list<NodeConstIt>& children_ref, NodeConstIt father_it)
    : AABBox(father_it)
{
    for (NodeConstIt adding_child : children_ref)
    {
        RLSU_ASSERT(NodeItIsValid(adding_child));
        AddChild(adding_child);
    }
}


AABContainer::AABContainer(NodeConstIt father_it)
    : AABBox(father_it)
    , children_()
{}


void AABContainer::AddChild(NodeConstIt new_child_it)
{
    children_.push_back(new_child_it);

    UpdateSizeAccordChild(new_child_it);
}

void AABContainer::AbandonChild(NodeConstIt unwanted_child_it)
{
    RLSU_ASSERT(ContainsChild(unwanted_child_it));

    children_.erase(std::find(children_.begin(), children_.end(), unwanted_child_it));
}


void AABContainer::UpdateSizeAccordChild(NodeConstIt child_it)
{
    RLSU_ASSERT(ContainsChild(child_it));

    if (!first_child_added_)
    {
        p0_ = child_it->get()->GetP0();
        p1_ = child_it->get()->GetP1();
        first_child_added_ = true;
    }

    this->SetP0(MinAxisPoint(this->GetP0(), child_it->get()->GetP0()));
    this->SetP1(MaxAxisPoint(this->GetP1(), child_it->get()->GetP1()));
}


bool AABContainer::ContainsChild(NodeConstIt child_it) const
{
    return (std::find(children_.begin(), children_.end(), child_it) != children_.end());
}


bool AABContainer::NoVolume() const
{
    Math::Vector3 diag = p1_ - p0_;

    return (Math::DoubleZero(diag.GetX() * diag.GetY() * diag.GetZ()));
}


AABLeaf::AABLeaf(const GeomObj& inscribed, NodeConstIt father_ptr)
    : AABBox(father_ptr)
    , inscribed_(inscribed)
{
    ASSERT_HANDLE(inscribed.Assert());

    switch (inscribed_.WhoAmI()) {
        case POINT3    : ERROR_HANDLE(BuildFromPoint_   ());  break;
        case LINESECT3 : ERROR_HANDLE(BuildFromLinesect_());  break;
        case TRIANGLE3 : ERROR_HANDLE(BuildFromTriangle_());  break;
        
        default        : RLSU_ERROR("invalid owner type = '{}'", ObjTypeStr(inscribed_.WhoAmI()));        
    }
}


void AABLeaf::BuildFromPoint_()
{
    RLSU_ASSERT(inscribed_.WhoAmI() == POINT3);

    const Primitives::Point3& inscribed_ref = static_cast<const Geometry::Primitives::Point3&>(inscribed_);

    p0_.SetX(inscribed_ref.GetX() - Math::CmpEps);
    p0_.SetY(inscribed_ref.GetY() - Math::CmpEps);
    p0_.SetZ(inscribed_ref.GetZ() - Math::CmpEps);

    p1_.SetX(inscribed_ref.GetX() + Math::CmpEps);
    p1_.SetY(inscribed_ref.GetY() + Math::CmpEps);
    p1_.SetZ(inscribed_ref.GetZ() + Math::CmpEps);
}


void AABLeaf::BuildFromLinesect_()
{
    RLSU_WARNING("building from linesect~~!!");

    RLSU_ASSERT(inscribed_.WhoAmI() == LINESECT3);

    auto inscribed_linesect = static_cast<const Geometry::Shapes::Linesect3& >(inscribed_);
    
    double x0_ = std::min(inscribed_linesect.GetPoint1().GetX(), inscribed_linesect.GetPoint2().GetX());
    double x1_ = std::max(inscribed_linesect.GetPoint1().GetX(), inscribed_linesect.GetPoint2().GetX());

    double y0_ = std::min(inscribed_linesect.GetPoint1().GetY(), inscribed_linesect.GetPoint2().GetY());
    double y1_ = std::max(inscribed_linesect.GetPoint1().GetY(), inscribed_linesect.GetPoint2().GetY());

    double z0_ = std::min(inscribed_linesect.GetPoint1().GetZ(), inscribed_linesect.GetPoint2().GetZ());
    double z1_ = std::max(inscribed_linesect.GetPoint1().GetZ(), inscribed_linesect.GetPoint2().GetZ());

    p0_.SetX(x0_);
    p0_.SetY(y0_);
    p0_.SetZ(z0_);

    p1_.SetX(x1_);
    p1_.SetY(y1_);
    p1_.SetZ(z1_);
}


void AABLeaf::BuildFromTriangle_()
{
    RLSU_ASSERT(inscribed_.WhoAmI() == TRIANGLE3);
    
    auto inscribed_triangle = static_cast<const Geometry::Shapes::Triangle3&>(inscribed_);

    double tr_x1 = inscribed_triangle.GetPoint1().GetX();
    double tr_x2 = inscribed_triangle.GetPoint2().GetX();
    double tr_x3 = inscribed_triangle.GetPoint3().GetX();

    double tr_y1 = inscribed_triangle.GetPoint1().GetY();
    double tr_y2 = inscribed_triangle.GetPoint2().GetY();
    double tr_y3 = inscribed_triangle.GetPoint3().GetY();

    double tr_z1 = inscribed_triangle.GetPoint1().GetZ();
    double tr_z2 = inscribed_triangle.GetPoint2().GetZ();
    double tr_z3 = inscribed_triangle.GetPoint3().GetZ();


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