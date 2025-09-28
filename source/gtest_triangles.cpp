#include <gtest/gtest.h>

#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"

TEST(TrianglesTest_one_plane, 1)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 2)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0  + 20, 0  + 20, 0}, 
                                    {10 + 20, 0  + 20, 0}, 
                                    {0  + 20, 10 + 20, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_one_plane, 3)  // one side matches
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,   0}, 
                                    {10, 0,   0}, 
                                    {0,  -10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 4)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,   0}, 
                                    {13, 0,   0}, 
                                    {0,  -10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 5)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({3,  0,   0}, 
                                    {13, 0,   0}, 
                                    {0,  -10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 6)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({3,  0,   0}, 
                                    {8 , 0,   0}, 
                                    {0,  -10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 7)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   0}, 
                                    {-10, 0,   0}, 
                                    {0,   -10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 8)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   0}, 
                                    {13,  0,   0}, 
                                    {0,   13,  0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 9)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   0}, 
                                    {10,  0,   0}, 
                                    {0,   16,  0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 10)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 5,  0}, 
                                    {0,  16, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 11)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({-5, 0,  0}, 
                                    {10, 5,  0}, 
                                    {0,  16, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 12)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({-5, 0,  0}, 
                                    {10, 5,  0}, 
                                    {0,  8, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 13)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({-5, 0,  0}, 
                                    {10, 5,  0}, 
                                    {1,  8, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 14)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({-5, -5,  0}, 
                                    {15,  0,  0}, 
                                    {0,  15, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}



TEST(TrianglesTest_parallel_planes, 1)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  5}, 
                                    {10, 0,  5}, 
                                    {0,  10, 5}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}



TEST(TrianglesTest_cross_planes, 1)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,   5}, 
                                    {15, 0,   5}, 
                                    {0,  -15, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 2)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0, 10}, 
                                    {15, 0, 10}, 
                                    {0,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}