#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/point3.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/shapes/shapes.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"
#include <cstddef>
#include <string>
#include <random>
#include <vector>
#include <filesystem>

#include "BvhTestGen/bvh_test_gen.hpp"

namespace Geometry::BvhTestGen {

static Geometry::Math::Point3 GenerateRandomPointInPlane(const Math::Point3& area_point, double area, const Math::Vector3& plane_normal, std::mt19937& gen)
{
    std::uniform_real_distribution<double> dist(-area, area);

    Math::Vector3 u;
    Math::Vector3 v;
    Math::Vector3 normal = plane_normal.Normalized();
    
    if (std::abs(normal.GetX()) < 0.9)
    {
        u = Math::Vector3(1, 0, 0);
    }

    else
    {
        u = Math::Vector3(0, 1, 0);
    }

    u = (u - normal * (u * normal)).Normalized(); 
    v = normal ^ u;
    
    Geometry::Math::Point3 random_point = area_point + u * dist(gen) + v * dist(gen);

    return random_point;
}

static Geometry::GeomObjUniqPtr GenerateRandomTriangleInPlane(const Math::Point3& area_point, double area, const Math::Vector3& plane_normal, std::mt19937& gen)
{
    Geometry::Math::Point3 a = GenerateRandomPointInPlane(area_point, area, plane_normal, gen);
    Geometry::Math::Point3 b = GenerateRandomPointInPlane(area_point, area, plane_normal, gen);
    Geometry::Math::Point3 c = GenerateRandomPointInPlane(area_point, area, plane_normal, gen);

    return Geometry::Shapes::Triangle3::BuildGeomObj(a, b, c);
}

static void GenerateRandomTriangles(std::vector<Geometry::GeomObjUniqPtr>& triangles, const size_t triangles_num, const size_t triangles_num_in_plane,
                                    double characteristic_size = 1.0, double area_x = 100.0, double area_y = 100.0, double area_z = 100.0)
{   
    Geometry::Math::Vector3 plane_normal = {1, 0, 0};

    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    std::uniform_real_distribution<double> plane_normal_axis_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> space_dist_x(0, area_x);
    std::uniform_real_distribution<double> space_dist_y(0, area_y);
    std::uniform_real_distribution<double> space_dist_z(0, area_z);
    
    for (size_t i = 0, cur_triangles_num_in_plane = 0; i < triangles_num; i++, cur_triangles_num_in_plane++)
    {
        if (cur_triangles_num_in_plane >= triangles_num_in_plane)
        {
            plane_normal = {plane_normal_axis_dist(gen), plane_normal_axis_dist(gen), plane_normal_axis_dist(gen)};
            cur_triangles_num_in_plane = 0;
        }

        Math::Point3 rand_area_point = {space_dist_x(gen), space_dist_y(gen), space_dist_z(gen)};

        triangles.push_back(GenerateRandomTriangleInPlane(rand_area_point, characteristic_size, plane_normal, gen));
    }
}


static std::vector<bool> GenerateAnswer(const std::vector<Geometry::GeomObjUniqPtr>& triangles)
{
    std::vector<bool> answer(triangles.size());

    for (size_t i = 0; i < triangles.size(); i++)
    {
        for (size_t j = i + 1; j < triangles.size(); j++)
        {
            if (Geometry::MathEngine::Interact(*triangles[i], *triangles[j])->CollisionCode() != MathEngine::NOTHING)
            {
                answer[i] = true;
                answer[j] = true;
                break;
            }
        }
    }

    return answer;
}

void GenerateTest(const std::string& tests_dir, const size_t test_num, const size_t triangles_num, const size_t triangles_num_in_plane,
                 double characteristic_size, double area_x, double area_y, double area_z)
{
    std::string task_dir   = tests_dir + "/" + BVH_TASKS_FOLDER;
    std::string answer_dir = tests_dir + "/" + BVH_ANSWERS_FOLDER;
    
    if (!std::filesystem::exists(task_dir))
        std::filesystem::create_directories(task_dir);

    if (!std::filesystem::exists(answer_dir))
        std::filesystem::create_directories(answer_dir);

    std::string task_file_name   = tests_dir + "/" + BVH_TASKS_FOLDER   + "/" + std::to_string(test_num) + ".dat";
    std::string answer_file_name = tests_dir + "/" + BVH_ANSWERS_FOLDER + "/" + std::to_string(test_num) + ".dat";

    std::ofstream task_file  (task_file_name);
    std::ofstream answer_file(answer_file_name);

    RLSU_VERIFY(task_file.is_open(),   "filename = '{}'", task_file_name);
    RLSU_VERIFY(answer_file.is_open(), "filename = '{}'", answer_file_name);

    std::vector<Geometry::GeomObjUniqPtr> triangles;
    GenerateRandomTriangles(triangles, triangles_num, triangles_num_in_plane, 
                            characteristic_size, area_x, area_y, area_z);

    std::vector<bool> answer = GenerateAnswer(triangles);

    task_file << triangles_num << std::endl;

    for (const auto& obj : triangles)
    {
        switch (obj->WhoAmI()) {
        case TRIANGLE3:
        {
            const Geometry::Shapes::Triangle3* triangle = static_cast<const Geometry::Shapes::Triangle3*>(obj.get());

            task_file << triangle->GetPoint1().GetX() << " "
                      << triangle->GetPoint1().GetY() << " "
                      << triangle->GetPoint1().GetZ() << " "
                      << "   "
                      << triangle->GetPoint2().GetX() << " "
                      << triangle->GetPoint2().GetY() << " "
                      << triangle->GetPoint2().GetZ() << " "
                      << "   "
                      << triangle->GetPoint3().GetX() << " "
                      << triangle->GetPoint3().GetY() << " "
                      << triangle->GetPoint3().GetZ() << " "
                      << std::endl;

            break;
        }

        case LINESECT3:
        {
            const Geometry::Shapes::Linesect3* linesect = static_cast<const Geometry::Shapes::Linesect3*>(obj.get());

            task_file << linesect->GetPoint1().GetX() << " "
                      << linesect->GetPoint1().GetY() << " "
                      << linesect->GetPoint1().GetZ() << " "
                      << "   "
                      << linesect->GetPoint2().GetX() << " "
                      << linesect->GetPoint2().GetY() << " "
                      << linesect->GetPoint2().GetZ() << " "
                      << "   "
                      << 0 << " "
                      << 0 << " "
                      << 0 << " "
                      << std::endl;

            break;
        }

        case POINT3:
        {
            const Geometry::Primitives::Point3* point = static_cast<const Geometry::Primitives::Point3*>(obj.get());

            task_file << point->GetX() << " "
                      << point->GetY() << " "
                      << point->GetZ() << " "
                      << "   "
                      << 0 << " "
                      << 0 << " "
                      << 0 << " "
                      << "   "
                      << 0 << " "
                      << 0 << " "
                      << 0 << " "
                      << std::endl;

            break;
        }

        default:
            RLSU_ERROR("invalid triangle type");
        }
    }

    task_file.close();


    for (size_t i = 0; i < answer.size(); i++)
    {
        if (answer[i] == true)
        {
            answer_file << i << std::endl;
        }
    }

    answer_file.close();
}



} // namespace Geometry::BvhTestGen
