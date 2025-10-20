#include <cstddef>
#include <iostream>
#include <string>
#include "BvhTestGen/bvh_test_gen.hpp"
#include "RLogSU/logger.hpp"


int main(const int argc, const char* argv[])
{
    // if (!(std::cin >> tests_dir))
    // {
    //     RLSU_WARNING("No input data, creating default test!");
    //     Geometry::BvhTestGen::GenerateTest(Geometry::BvhTestGen::DEFAULT_TESTS_DIR, 0, 30, Geometry::BvhTestGen::TRIANGLES_IN_PLANE_NUM);

    //     return 0;
    // }

    std::string tests_dir;
    if (!(std::cin >> tests_dir))  RLSU_ERROR("cannot read tests_folder");

    size_t tests_num = 0;
    if (!(std::cin >> tests_num))  RLSU_ERROR("cannot read tests_num");

    for (size_t test_num = 0; test_num < tests_num; test_num++)
    {
        double area_x = 0;
        double area_y = 0;
        double area_z = 0;

        size_t triangles_num = 0;
        double characteristic_size = 0;

        if (!(std::cin >> area_x))               RLSU_ERROR("cannot read area_x");
        if (!(std::cin >> area_y))               RLSU_ERROR("cannot read area_y");
        if (!(std::cin >> area_z))               RLSU_ERROR("cannot read area_z");

        if (!(std::cin >> triangles_num))        RLSU_ERROR("cannot read triangles_num");
        if (!(std::cin >> characteristic_size))  RLSU_ERROR("cannot read characteristic_size");

        Geometry::BvhTestGen::GenerateTest(tests_dir, test_num, triangles_num, Geometry::BvhTestGen::TRIANGLES_IN_PLANE_NUM, 
                                            characteristic_size, area_x, area_y, area_z);
    }

    return 0;
}