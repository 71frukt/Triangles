#pragma once

#include <cstddef>
#include <iostream>
#include <vector>
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include <variant>

namespace Geometry::MathEngine {
    

class BvhTree
{
public:
    BvhTree(const std::vector<const GeomObj*>& objects, const size_t box_max_capa_ = 6);

private:
    std::vector<std::unique_ptr<AABBox>> nodes_;
    AABContainer* root_;

    size_t box_max_capa_;

    void BuildRootBox_();
    void SplitIntoBoxes_(AABBox& cur_box);
};


} // namespace Geometry::MathEngine