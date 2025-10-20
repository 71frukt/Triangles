#pragma once

#include <cstddef>
#include <iostream>
#include <string>

namespace Geometry::BvhTestGen {

const std::string DEFAULT_TESTS_DIR  = "def_bvh_tests";

const std::string BVH_TASKS_FOLDER   = "tasks";
const std::string BVH_ANSWERS_FOLDER = "answers";

const size_t TRIANGLES_IN_PLANE_NUM  = 2;


void GenerateTest(const std::string& tests_dir, const size_t test_num, const size_t triangles_num, const size_t triangles_num_in_plane,
                  double characteristic_size = 5.0, double area_x = 10.0, double area_y = 10.0, double area_z = 10.0);

}