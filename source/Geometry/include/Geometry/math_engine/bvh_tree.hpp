#pragma once

#include <cstddef>
#include <vector>
#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/graph.hpp"

namespace Geometry::MathEngine {
    

class BvhTree
{
public:
    BvhTree(const std::vector<const GeomObj*>& objects, const size_t box_max_capa = 3);

    void Assert() const;
    void Dump()   const;

private:
    std::list<std::unique_ptr<const AABBox>> nodes_;
    AABContainer* root_;

    size_t box_max_capa_;

    void AdoptChildToFather(NodeConstIt father_it, NodeConstIt new_child_it);   // adds child to father and father to child
    // void MoveChildToOtherContainer_(NodeConstIt child_it, NodeConstIt source_cont_it, NodeConstIt& target_cont_it);

    void BuildRootBox_();
    void SplitIntoBoxes_(AABContainer& cur_cont);
    void CleanUp_(AABContainer& cont);

    void EraseNode_               (const AABContainer& erasing_node );
    void ResolveDegradedContainer_(      AABContainer& degraded_cont);
    void ResolveEmptyContainer_   (      AABContainer& empty_cont   );

    void AddNodeEdges_(RLSU::Graphics::Graph& graph, const AABBox& node_tree) const;
    void AddConfiduredGraphNode_(RLSU::Graphics::Graph& graph, const AABBox& tree_node) const;
};


} // namespace Geometry::MathEngine