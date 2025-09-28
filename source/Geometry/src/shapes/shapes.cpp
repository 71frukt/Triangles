#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"

namespace Geometry::Shapes {


void Linesect3::Assert() const
{
    RLSU_ASSERT(MathEngine::Interact(point1_, line_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point2_, line_)->CollisionCode() == MathEngine::LIES_IN);
}

bool Linesect3::operator== (const Linesect3& other) const
{
    return ((this->point1_ == other.point1_ && this->point2_ == other.point2_)
          || this->point1_ == other.point1_ && this->point2_ == other.point2_);
}


// void Halfint3::Assert() const
// {
//     RLSU_ASSERT(MathEngine::Interact(close_end_, line_)->CollisionCode() == MathEngine::LIES_IN);
//     RLSU_ASSERT(MathEngine::Interact(open_end_ , line_)->CollisionCode() == MathEngine::LIES_IN);
// }


void Triangle3::Assert() const
{
    ASSERT_HANDLE(side1_.Assert());
    ASSERT_HANDLE(side2_.Assert());
    ASSERT_HANDLE(side3_.Assert());
    ASSERT_HANDLE(plane_.Assert());

    RLSU_ASSERT(point1_ != point2_ && point1_ != point3_ && point2_ != point3_);

    RLSU_ASSERT(MathEngine::Interact(point1_, side1_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point1_, side3_)->CollisionCode() == MathEngine::LIES_IN);
    
    RLSU_ASSERT(MathEngine::Interact(point2_, side1_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point2_, side2_)->CollisionCode() == MathEngine::LIES_IN);
    
    RLSU_ASSERT(MathEngine::Interact(point3_, side2_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point3_, side3_)->CollisionCode() == MathEngine::LIES_IN);
}


Triangle3::Triangle3(const Primitives::Point3& point1, const Primitives::Point3& point2, const Primitives::Point3& point3)
    : point1_(point1)
    , point2_(point2)
    , point3_(point3)

    , side1_ (ERROR_HANDLE(Linesect3(point1, point2)))
    , side2_ (ERROR_HANDLE(Linesect3(point2, point3)))
    , side3_ (ERROR_HANDLE(Linesect3(point3, point1)))

    , plane_(point1, point2, point3)
{ 
    ASSERT_HANDLE(Assert());

    RLSU_VERIFY(point1 != point2 && point1 != point3 && point2 != point3);
}

Triangle3::Triangle3(const Primitives::Line3& line1 , const Primitives::Line3& line2 , const Primitives::Line3& line3 )
    : point1_(ERROR_HANDLE(MathEngine::Interact(line3, line1)->Intersect()))
    , point2_(ERROR_HANDLE(MathEngine::Interact(line1, line2)->Intersect()))
    , point3_(ERROR_HANDLE(MathEngine::Interact(line2, line3)->Intersect()))

    , side1_ (point1_, point2_)
    , side2_ (point2_, point3_)
    , side3_ (point3_, point1_)

    , plane_(point1_, point2_, point3_)
{ 
    ASSERT_HANDLE(Assert());

    RLSU_VERIFY(side1_  != side2_  && side1_  != side3_  && side2_  != side3_);
}

}