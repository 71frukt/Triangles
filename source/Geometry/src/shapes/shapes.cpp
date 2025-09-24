#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"

namespace Geometry::Shapes {

Triangle3::Triangle3(const Primitives::Point3& point1, const Primitives::Point3& point2, const Primitives::Point3& point3)
    : point1_(point1)
    , point2_(point2)
    , point3_(point3)

    , line1_ (point1, point2 - point1)
    , line2_ (point2, point3 - point2)
    , line3_ (point3, point1 - point3)

    , plane_(point1, point2, point3)
{ }

Triangle3::Triangle3(const Primitives::Line3& line1 , const Primitives::Line3& line2 , const Primitives::Line3& line3 )
    : line1_ (line1)
    , line2_ (line2)
    , line3_ (line3)

    , point1_(MathEngine::Interact(line3, line1)->Intersect())
    , point2_(MathEngine::Interact(line1, line2)->Intersect())
    , point3_(MathEngine::Interact(line2, line3)->Intersect())

    , plane_(point1_, point2_, point3_)
{ }



void Linesect3::Assert() const
{
    RLSU_ASSERT(MathEngine::Interact(point1_, line_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point2_, line_)->CollisionCode() == MathEngine::LIES_IN);
}

void Triangle3::Assert() const
{
    RLSU_ASSERT(MathEngine::Interact(point1_, line1_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point1_, line3_)->CollisionCode() == MathEngine::LIES_IN);
    
    RLSU_ASSERT(MathEngine::Interact(point2_, line1_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point2_, line2_)->CollisionCode() == MathEngine::LIES_IN);
    
    RLSU_ASSERT(MathEngine::Interact(point3_, line2_)->CollisionCode() == MathEngine::LIES_IN);
    RLSU_ASSERT(MathEngine::Interact(point3_, line3_)->CollisionCode() == MathEngine::LIES_IN);
}

}