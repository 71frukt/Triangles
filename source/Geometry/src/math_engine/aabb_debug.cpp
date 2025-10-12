#include "Geometry/math_engine/aabb.hpp"

namespace Geometry::MathEngine {


void AABBox::Assert() const
{
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

    for (auto child : children_)
    {
        // checking that the shape is at least PARTIALLY in the box

        RLSU_ASSERT(child->GetP1().GetX() >= p0_.GetX(), "cont->p0_X = {}, child->p1_X = {}", p0_.GetX(), child->GetP1().GetX());
        RLSU_ASSERT(child->GetP1().GetY() >= p0_.GetY(), "cont->p0_Y = {}, child->p1_Y = {}", p0_.GetY(), child->GetP1().GetY());
        RLSU_ASSERT(child->GetP1().GetZ() >= p0_.GetZ(), "cont->p0_Z = {}, child->p1_Z = {}", p0_.GetZ(), child->GetP1().GetZ());
        
        RLSU_ASSERT(child->GetP0().GetX() <= p1_.GetX(), "cont->p1_X = {}, child->p0_X = {}", p1_.GetX(), child->GetP0().GetX());
        RLSU_ASSERT(child->GetP0().GetY() <= p1_.GetY(), "cont->p1_Y = {}, child->p0_Y = {}", p1_.GetY(), child->GetP0().GetY());
        RLSU_ASSERT(child->GetP0().GetZ() <= p1_.GetZ(), "cont->p1_Z = {}, child->p0_Z = {}", p1_.GetZ(), child->GetP0().GetZ());
    }
}

void AABBox::Dump() const
{
    if (typeid(*this) == typeid(AABContainer))
        RLSU_LOG("AABContainer:\n");

    else if (typeid(*this) == typeid(AABLeaf))
        RLSU_LOG("AABLeaf:\n");

    else
        RLSU_LOG("Invalid AABBox:\n");

    RLSU_LOG("\tx: [{:.2f}, {:.2f}] \n"
                     "\ty: [{:.2f}, {:.2f}] \n"
                     "\tz: [{:.2f}, {:.2f}] \n\n",
        
                GetP0().GetX(), GetP1().GetX(),
                GetP0().GetY(), GetP1().GetY(),
                GetP0().GetZ(), GetP1().GetZ()
            );
}


} // namespace Geometry::MathEngine