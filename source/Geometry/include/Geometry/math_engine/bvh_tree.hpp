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
    ~BvhTree();

    [[nodiscard]] std::vector<bool> GetIntersections() const;

    void Assert() const;
    void Dump()   const;

private:
    std::list<AABBox*> nodes_;
    AABContainer* root_;

    size_t box_max_capa_;

    void BuildRootBox_();
    void SplitIntoBoxes_(AABContainer* cur_cont);

    void EraseNode_               (const AABBox      * erasing_node );
    void ResolveDegradedContainer_(      AABContainer* degraded_cont);
    void ResolveEmptyContainer_   (      AABContainer* empty_cont   );

    // void GetIntersectionsInContainer_(std::vector<std::vector<bool>>& res_table, const AABContainer* cont) const;
    void GetLeafIntersectionInContainer_(const AABLeaf& leaf, const AABContainer& cont, std::vector<bool>& intersections) const;

    //----------debug-------------------------------------------------------------------------
    void AddNodeEdges_          (RLSU::Graphics::Graph& graph, const AABBox* node_tree) const;
    void AddConfiduredGraphNode_(RLSU::Graphics::Graph& graph, const AABBox* tree_node) const;
    //----------------------------------------------------------------------------------------

};


} // namespace Geometry::MathEngine