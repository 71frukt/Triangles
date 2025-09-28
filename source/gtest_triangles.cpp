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


TEST(TrianglesTest_one_plane, 3a)
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


TEST(TrianglesTest_one_plane, 3b)
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


TEST(TrianglesTest_one_plane, 3c)
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


TEST(TrianglesTest_one_plane, 3d)
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


TEST(TrianglesTest_one_plane, 3f)
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


TEST(TrianglesTest_one_plane, 3x)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({-5,  0,   0}, 
                                    {-15, 0,   0}, 
                                    {0,   -10, 0}));
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_one_plane, 4a)
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


TEST(TrianglesTest_one_plane, 4b)
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


TEST(TrianglesTest_one_plane, 4c)
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


TEST(TrianglesTest_one_plane, 5a)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0, 0,  0}, 
                                    {10, 5,  0}, 
                                    {0,  16, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}

TEST(TrianglesTest_one_plane, 5b)
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


TEST(TrianglesTest_one_plane, 5c)
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


TEST(TrianglesTest_one_plane, 5d)
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


TEST(TrianglesTest_one_plane, 6a)
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


TEST(TrianglesTest_one_plane, 6x)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {10, 0,  0}, 
                                    {0,  10, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({6,  6,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
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



TEST(TrianglesTest_cross_planes, 1a)
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


TEST(TrianglesTest_cross_planes, 1b)
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


TEST(TrianglesTest_cross_planes, 1c)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0, 10}, 
                                    {15, 0, 0}, 
                                    {0,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1d)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {15,  0, 0}, 
                                    {-5,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1f)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  0, 0}, 
                                    {-5,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1g)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {18,  0, 0}, 
                                    {-5,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1xa)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  0, 10}, 
                                    {-5,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 1xb)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {-15, 0, 0}, 
                                    {-5,  0, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 2a)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {18,  0, 0}, 
                                    {0,   5, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2b)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  5, 0}, 
                                    {0,   5, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2c)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  5, -5}, 
                                    {0,   5, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2d)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  0, -5}, 
                                    {0,   5, -5}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2f)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  2, -5}, 
                                    {2,   5, -5}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2g)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {10,  2, -5}, 
                                    {-10, 5, -5}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3a)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,  10}, 
                                    {10,  -5, -5}, 
                                    {-10, 5,  -5}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3b)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,  10}, 
                                    {10,  -5, 0}, 
                                    {-10, 5,  0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3c)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,  10}, 
                                    {-5,  -5, 0}, 
                                    {10, 10,  0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3d)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0, 10}, 
                                    {2,   2, 0}, 
                                    {10, 10, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3xa)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   10}, 
                                    {-2,  -2,  0}, 
                                    {-10, -10, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 3xb)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   10}, 
                                    {-5,  0,   0}, 
                                    {-10, -10, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 3xc)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   10}, 
                                    {-10, 0,   0}, 
                                    { 0,  -10, 0}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 3xd)
{
    Geometry::Shapes::Triangle3 tr1 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,  0,  0}, 
                                    {15, 0,  0}, 
                                    {0,  15, 0}));
    
    Geometry::Shapes::Triangle3 tr2 = ERROR_HANDLE(
        Geometry::Shapes::Triangle3({0,   0,   10}, 
                                    {-10, 5,   -5}, 
                                    { 5,  -10, -5}));

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(tr1, tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}