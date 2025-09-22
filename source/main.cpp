
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "RLogSU/logger.hpp"


int main()
{
    RLSU_INFO("START");

    // Geometry::Primitives::Point3 point1(0, 0, 9);
    // Geometry::Primitives::Point3 point2(1, 1, 1);

    // Geometry::Primitives::Line3 line1({1, 0, 0}, {1, 0, 0});
    // Geometry::Primitives::Line3 line2({1, 0, 0}, {1, 0, 0});

    // auto interactor = Geometry::MathEngine::Interact(line1, line2);
    // auto fig1 = interactor->Intersect();

    // RLSU_INFO("inter1 = {}", Geometry::MathEngine::CollisionCodeStr(interactor->CollisionCode()));
    // RLSU_INFO("dist1  = {}", interactor->Distance());

    // RLSU_DUMP(fig1->Dump("fig1"));
    
    // auto fig2 = Geometry::MathEngine::Interact(*fig1, point1)->Intersect();
    // RLSU_DUMP(fig2->Dump("fig2"));


    Geometry::Primitives::Plane3 plane1(0, 0, 1, 3);
    Geometry::Primitives::Plane3 plane2(0, 1, 1, 3);

    auto interactor = Geometry::MathEngine::Interact(plane1, plane2);
    auto fig1 = interactor->Intersect();

    RLSU_INFO("inter1 = {}", Geometry::MathEngine::CollisionCodeStr(interactor->CollisionCode()));
    RLSU_INFO("dist1  = {}", interactor->Distance());

    plane1.Dump("plane1");
    plane2.Dump("plane2");
    
    fig1->Dump("plane x plane");

    RLSU_INFO("END");
}