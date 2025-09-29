#include <gtest/gtest.h>

#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

TEST(TrianglesTest_one_plane, 1)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
                                    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 2)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0  + 20, 0  + 20, 0}, 
                                                  {10 + 20, 0  + 20, 0}, 
                                                  {0  + 20, 10 + 20, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_one_plane, 3a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,   0}, 
                                                  {10, 0,   0}, 
                                                  {0,  -10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 3b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,   0}, 
                                                  {13, 0,   0}, 
                                                  {0,  -10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 3c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({3,  0,   0}, 
                                                  {13, 0,   0}, 
                                                  {0,  -10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 3d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({3,  0,   0}, 
                                                  {8 , 0,   0}, 
                                                  {0,  -10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 3f)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   0}, 
                                                  {-10, 0,   0}, 
                                                  {0,   -10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 3x)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5,  0,   0}, 
                                                  {-15, 0,   0}, 
                                                  {0,   -10, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_one_plane, 4a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   0}, 
                                                  {13,  0,   0}, 
                                                  {0,   13,  0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 4b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   0}, 
                                                  {10,  0,   0}, 
                                                  {0,   16,  0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 4c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 5,  0}, 
                                                  {0,  16, 0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 5a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, 0,  0}, 
                                                  {10, 5,  0}, 
                                                  {0,  16, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}

TEST(TrianglesTest_one_plane, 5b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5, 0,  0}, 
                                                  {10, 5,  0}, 
                                                  {0,  16, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 5c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5, 0,  0}, 
                                                  {10, 5,  0}, 
                                                  {0,  8, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 5d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5, 0,  0}, 
                                                  {10, 5,  0}, 
                                                  {1,  8, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 6a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5, -5,  0}, 
                                                  {15,  0,  0}, 
                                                  {0,  15, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_one_plane, 6x)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({6,  6,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_parallel_planes, 1)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  5}, 
                                                  {10, 0,  5}, 
                                                  {0,  10, 5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}



TEST(TrianglesTest_cross_planes, 1a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,   5}, 
                                                  {15, 0,   5}, 
                                                  {0,  -15, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 1b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 10}, 
                                                  {15, 0, 10}, 
                                                  {0,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 10}, 
                                                  {15, 0, 0}, 
                                                  {0,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {15,  0, 0}, 
                                                  {-5,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1f)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  0, 0}, 
                                                  {-5,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1g)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {18,  0, 0}, 
                                                  {-5,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 1xa)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  0, 10}, 
                                                  {-5,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 1xb)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {-15, 0, 0}, 
                                                  {-5,  0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 2a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {18,  0, 0}, 
                                                  {0,   5, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  5, 0}, 
                                                  {0,   5, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  5, -5}, 
                                                  {0,   5, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  0, -5}, 
                                                  {0,   5, -5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2f)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  2, -5}, 
                                                  {2,   5, -5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 2g)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {10,  2, -5}, 
                                                  {-10, 5, -5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,  10}, 
                                                  {10,  -5, -5}, 
                                                  {-10, 5,  -5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,  10}, 
                                                  {10,  -5, 0}, 
                                                  {-10, 5,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,  10}, 
                                                  {-5,  -5, 0}, 
                                                  {10, 10,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0, 10}, 
                                                  {2,   2, 0}, 
                                                  {10, 10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_cross_planes, 3xa)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   10}, 
                                                  {-2,  -2,  0}, 
                                                  {-10, -10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 3xb)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   10}, 
                                                  {-5,  0,   0}, 
                                                  {-10, -10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 3xc)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   10}, 
                                                  {-10, 0,   0}, 
                                                  { 0,  -10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_cross_planes, 3xd)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {15, 0,  0}, 
                                                  {0,  15, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,   0,   10}, 
                                                  {-10, 5,   -5}, 
                                                  { 5,  -10, -5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_degenerate, 1a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {0,  0,  0}, 
                                                  {0,  0,  0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {0,  0,  0}, 
                                                  {0,  0,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::EQUAL);
}


TEST(TrianglesTest_degenerate, 2a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
                                                  
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {0,  0,  0}, 
                                                  {0,  0,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::LIES_IN);
}


TEST(TrianglesTest_degenerate, 2b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});

    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({5,  0,  0}, 
                                                  {5,  0,  0}, 
                                                  {5,  0,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::LIES_IN);
}


TEST(TrianglesTest_degenerate, 2c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
                                                  
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5,  0,  0}, 
                                                  {-5,  0,  0}, 
                                                  {-5,  0,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_degenerate, 2d)
{
        
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});

    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5,  -5,  0}, 
                                                  {-5,  -5,  0}, 
                                                  {-5,  -5,  0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_degenerate, 3a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({ 0,  0,  0}, 
                                                  {10,  0,  0}, 
                                                  {10,  0,  0});
    
    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}

TEST(TrianglesTest_degenerate, 3b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, 0,  0}, 
                                                  {0, 0,  10}, 
                                                  {0, 0,  5});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}

TEST(TrianglesTest_degenerate, 3c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-10, 0, 0}, 
                                                  {0,   0, 10}, 
                                                  {0,   0, 10});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_degenerate, 3d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({3, 3, 0}, 
                                                  {0, 0, 10}, 
                                                  {0, 0, 10});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_degenerate, 3f)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0,  0}, 
                                                  {10, 0,  0}, 
                                                  {0,  10, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({3, 3, -5}, 
                                                  {0, 0, 10}, 
                                                  {0, 0, 10});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_degenerate, 4a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, 0, 0}, 
                                                  {0, 0, 0}, 
                                                  {0, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::LIES_IN);
}


TEST(TrianglesTest_degenerate, 4b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-1, 0, 0}, 
                                                  {-1, 0, 0}, 
                                                  {-1, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}


TEST(TrianglesTest_degenerate, 4c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, 0, 0}, 
                                                  {0, 0, 0}, 
                                                  {0, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-10,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::LIES_IN);
}


TEST(TrianglesTest_degenerate, 5a)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::EQUAL);
}


TEST(TrianglesTest_degenerate, 5b)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-10,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_degenerate, 5c)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-10,  0, 0}, 
                                                  {20, 0, 0}, 
                                                  {20, 0, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::OVERLAP);
}


TEST(TrianglesTest_degenerate, 5d)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, 0,  0}, 
                                                  {0, 10, 0}, 
                                                  {0, 10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_degenerate, 5f)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, -5,  0}, 
                                                  {0, 10, 0}, 
                                                  {0, 10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}


TEST(TrianglesTest_degenerate, 5g)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5,  0, 0}, 
                                                  {10, 0, 0}, 
                                                  {10, 0, 0});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, -5,  0}, 
                                                  {0, 10, 0}, 
                                                  {0, 10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::CROSS);
}



TEST(TrianglesTest_degenerate, 5h)
{
    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({-5, 0, 5}, 
                                                  {10, 0, 5}, 
                                                  {10, 0, 5});
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, -5,  0}, 
                                                  {0, 10, 0}, 
                                                  {0, 10, 0});

    auto interactor = ERROR_HANDLE(Geometry::MathEngine::Interact(*tr1, *tr2));
    auto collision_code          = ERROR_HANDLE(interactor->CollisionCode());

    EXPECT_EQ(collision_code, Geometry::MathEngine::NOTHING);
}