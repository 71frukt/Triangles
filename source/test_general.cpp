
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/graph_appearance.hpp"
#include "RLogSU/logger.hpp"
#include <cstddef>
#include <iostream>
#include <ostream>
#include <stdexcept>


#include "RLogSU/graph.hpp"

int main()
{
    // size_t N = 0;
    // std::cin >> N;

    // Geometry::GeomObjUniqPtr triangles[N];

    // bool cross[N][N];
    // for (size_t i = 0; i < N; i++)
    //     for (size_t j = 0; j < N; j++)
    //         cross[i][j] = false;


    // for (size_t i = 0; i < N; i++)
    // {
    //     Geometry::Primitives::Point3 point1;
    //     Geometry::Primitives::Point3 point2;
    //     Geometry::Primitives::Point3 point3;

    //     double var = 0;

    //     std::cin >> var;    point1.SetX(var);
    //     std::cin >> var;    point1.SetY(var);
    //     std::cin >> var;    point1.SetZ(var);

    //     std::cin >> var;    point2.SetX(var);
    //     std::cin >> var;    point2.SetY(var);
    //     std::cin >> var;    point2.SetZ(var);

    //     std::cin >> var;    point3.SetX(var);
    //     std::cin >> var;    point3.SetY(var);
    //     std::cin >> var;    point3.SetZ(var);

    //     triangles[i] = Geometry::Shapes::Triangle3::BuildGeomObj(point1, point2, point3);
    // }

    // for (size_t i = 0; i < N; i++)
    // {
    //     for (size_t j = 0; j < N; j++)
    //     {
    //         if (Geometry::MathEngine::Interact(*triangles[i], *triangles[j])->CollisionCode() != Geometry::MathEngine::NOTHING)
    //             cross[i][j] = true;

    //         else
    //             cross[i][j] = false;
    //     }
    // }

    // for (size_t i = 0; i < N; i++)
    // {
    //     bool crosses = false;

    //     for (size_t j = 0; j < N; j++)
    //     {
    //         if (i == j)
    //             continue;
            
    //         if (cross[i][j] == true)
    //         {
    //             crosses = true;
    //             break;
    //         }
    //     }

    //     if (crosses)
    //         std::cout << i << " ";
    // }

    // std::cout << std::endl;



    Geometry::GeomObjUniqPtr tr1 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0, 0, 0}, 
                                                  {1, 0, 0}, 
                                                  {0, 1, 0});
    
    Geometry::GeomObjUniqPtr tr2 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({ 0,  0, 0}, 
                                                  {-1,  0, 0}, 
                                                  { 0, -1, 0});


    Geometry::GeomObjUniqPtr tr3 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({0 + 10, 0, 0}, 
                                                  {1 + 10, 0, 0}, 
                                                  {0 + 10, 1, 0});

    Geometry::GeomObjUniqPtr tr4 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({ 0 + 10,  0, 0}, 
                                                  {-1 + 10,  0, 0}, 
                                                  { 0 + 10, -1, 0});

    Geometry::GeomObjUniqPtr tr5 =  
        Geometry::Shapes::Triangle3::BuildGeomObj({ 0 + 10,  0 + 20, 0}, 
                                                  {-1 + 10,  0 + 20, 0}, 
                                                  { 0 + 10, -1 + 20, 0});

    std::vector<const Geometry::GeomObj*> objects;
    objects.push_back(tr1.get());
    objects.push_back(tr2.get());
    objects.push_back(tr3.get());
    objects.push_back(tr4.get());
    objects.push_back(tr5.get());

    try
    {
        Geometry::MathEngine::BvhTree bvh_tree(objects);

        RLSU_DUMP(bvh_tree.Dump());
    }
    catch(std::runtime_error a)
    {
        RLSU_ERROR("main() error: {}", a.what());
    }


    // RLSU::Graphics::Graph graph;

    // int node1 = 666;

    // RLSU::Graphics::Graph::Node graph_node1(node1);
    // graph_node1.SetLabel("graph_node1");

    // int node2 = 777;

    // RLSU::Graphics::Graph::Node graph_node2(node2);
    // graph_node2.SetLabel("graph_node2");

    // int node3 = 666;

    // RLSU::Graphics::Graph::Node graph_node3(node3);
    // graph_node3.SetLabel("graph_node3");

    // int node4 = 777;

    // RLSU::Graphics::Graph::Node graph_node4(node4);
    // graph_node4.SetLabel("graph_node4");

    // graph.AddNode(graph_node1);
    // graph.AddNode(graph_node2);
    // graph.AddNode(graph_node3);
    // graph.AddNode(graph_node4);


    // graph.AddEdge(graph_node1, graph_node2);
    // graph.AddEdge(graph_node1, graph_node3);
    // graph.AddEdge(graph_node3, graph_node4);
    // graph.AddEdge(graph_node3, graph_node4);
    // graph.AddEdge(graph_node3, graph_node4);
    // graph.AddEdge(graph_node3, graph_node4);

    // graph.LogGraph();


}