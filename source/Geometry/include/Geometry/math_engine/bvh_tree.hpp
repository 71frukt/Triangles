#pragma once

#include <cstddef>
#include <iostream>
#include <vector>
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/graph.hpp"
#include <variant>

namespace Geometry::MathEngine {
    

class BvhTree
{
public:
    BvhTree(const std::vector<const GeomObj*>& objects, const size_t box_max_capa = 3);

    void Assert() const;
    void Dump()   const;

private:
    std::vector<std::unique_ptr<AABBox>> nodes_;
    AABContainer* root_;

    size_t box_max_capa_;

    void BuildRootBox_();
    void SplitIntoBoxes_(AABContainer& cur_cont);

    void EraseNode_               (const AABContainer& erasing_node );
    void ResolveDegradedContainer_(      AABContainer& degraded_cont);
    void ResolveEmptyContainer_   (      AABContainer& empty_cont   );

    void AddNodeEdges_(RLSU::Graphics::Graph& graph, const AABBox& node_tree) const;
    void AddConfiduredGraphNode_(RLSU::Graphics::Graph& graph, const AABBox& tree_node) const;
};


} // namespace Geometry::MathEngine