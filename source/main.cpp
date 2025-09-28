
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"


int main()
{
    RLSU_INFO("START");

    try {        

        Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
            Geometry::Shapes::Triangle3({0, 0,  1}, 
                                        {10, 0, 1}, 
                                        {0, 10, 1}));
        
        Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
            Geometry::Shapes::Triangle3({0, 0,  1}, 
                                        {10, 0, 1}, 
                                        {0, 10, 1}));
        
        Geometry::Primitives::Line3 line({0.5, 0, 0}, Geometry::Primitives::Point3{0.5, 1, 0});

        tr1.Dump("tr1");
        tr2.Dump("tr2!");

        auto interactor = Geometry::MathEngine::Interact(tr1, tr2);
        auto fig1 = interactor->Intersect();
        
        RLSU_INFO("inter1 = {}", Geometry::MathEngine::CollisionCodeStr(interactor->CollisionCode()));
        // RLSU_INFO("dist1  = {}", interactor->Distance());
        
        // Geometry::Shapes::Linesect3 ls1({0, 0, 0}, Geometry::Primitives::Point3(0, 1, 0));
        // Geometry::Shapes::Linesect3 ls2({0, 0, 0}, Geometry::Primitives::Point3(0, 2, 0));
        // auto interactor = Geometry::MathEngine::Interact(ls1, ls2);
        // auto fig1 = interactor->Intersect();
        
        // RLSU_INFO("inter1 = {}", Geometry::MathEngine::CollisionCodeStr(interactor->CollisionCode()));
        // RLSU_INFO("dist1  = {}", interactor->Distance());
        
        // fig1->Dump("ls x ls");
    }
    catch (const std::runtime_error& _e) {
        RLSU_ERROR("what(): {}", _e.what());
        throw;
    }  

    RLSU_INFO("END");
}