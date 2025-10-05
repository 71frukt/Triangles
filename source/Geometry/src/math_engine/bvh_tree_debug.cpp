#include "RLogSU/graph.hpp"
#include "RLogSU/graph_appearance.hpp"

#include "Geometry/math_engine/bvh_tree.hpp"


namespace Geometry::MathEngine {


void BvhTree::Assert() const
{
    RLSU_ASSERT(root_);
    RLSU_ASSERT(typeid(*root_) == typeid(AABContainer));

    for (const std::unique_ptr<AABBox>& node : nodes_)
    {
        RLSU_ASSERT(node->father || node.get() == root_);
        node->Assert();
    }
}

void BvhTree::Dump() const
{
    RLSU::Graphics::Graph graph;

    for (const std::unique_ptr<AABBox>& node : nodes_)
    {
        AddConfiduredGraphNode_(graph, *node);
    }

    for (const std::unique_ptr<AABBox>& node : nodes_)
    {
        AddNodeEdges_(graph, *node);
    }

    graph.LogGraph();

    // for (size_t i = 0; i < nodes_.size(); i++)
    // {
    //     RLSU_LOG("[{}]  ", i);
    //     nodes_[i]->Dump();
    // }
}

void BvhTree::AddNodeEdges_(RLSU::Graphics::Graph& graph, const AABBox& tree_node) const
{
    if (typeid(tree_node) == typeid(AABContainer))
    {
        const AABContainer& container = static_cast<const AABContainer&>(tree_node);
        
        for (const AABBox* child_node_ptr : container.GetChildren())
        {
            RLSU_ASSERT(child_node_ptr);

            graph.AddEdge(&tree_node, child_node_ptr);
        }
    }
}

void BvhTree::AddConfiduredGraphNode_(RLSU::Graphics::Graph& graph, const AABBox& tree_node) const
{
    RLSU::Graphics::Graph::Node new_graph_node(&tree_node);

    int node_id = -1;
    for (int i = 0; i < nodes_.size(); i++)
    {
        if (nodes_[i].get() == &tree_node)
        {
            node_id = i;
            break;
        }
    }

    new_graph_node.SetLabel("{}\n"
                            "x: [{:.2f}, {:.2f}]\n"
                            "y: [{:.2f}, {:.2f}]\n"
                            "z: [{:.2f}, {:.2f}]\n",
        
                            node_id,
                            tree_node.GetP0().GetX(), tree_node.GetP1().GetX(),
                            tree_node.GetP0().GetY(), tree_node.GetP1().GetY(),
                            tree_node.GetP0().GetZ(), tree_node.GetP1().GetZ()
                        );

    if (typeid(tree_node) == typeid(AABContainer))
    {
        new_graph_node.SetShape(RLSU::Graphics::Shapes::BOX3D);
        new_graph_node.SetColor(RLSU::Graphics::Colors::AQUAMARINE);
    }

    else if (typeid(tree_node) == typeid(AABLeaf))
    {
        new_graph_node.SetShape(RLSU::Graphics::Shapes::BOX);
        new_graph_node.SetColor(RLSU::Graphics::Colors::SKYBLUE);
    }

    else
    {
        RLSU_ERROR("invalid node type");
        new_graph_node.SetShape(RLSU::Graphics::Shapes::DIAMOND);
        new_graph_node.SetColor(RLSU::Graphics::Colors::RED);
    }

    graph.AddNode(new_graph_node);
}

} // namespace Geometry::MathEngine