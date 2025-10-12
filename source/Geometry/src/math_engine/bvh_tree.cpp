#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/error_handler.hpp"

#include "RLogSU/logger.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <vector>

namespace Geometry::MathEngine {

BvhTree::BvhTree(const std::vector<const GeomObj*>& objects, const size_t box_max_capa)
    : box_max_capa_(box_max_capa)
    , root_(nullptr)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        ASSERT_HANDLE(objects[i]->Assert());

        AABLeaf* new_leaf = new AABLeaf(objects[i]); 

        nodes_.push_back(new_leaf);
    }

    ERROR_HANDLE(BuildRootBox_());

    RLSU_ASSERT(root_);
    RLSU_DUMP(Dump(), "build_root");

    ERROR_HANDLE(SplitIntoBoxes_(root_));

    ASSERT_HANDLE(Assert());
}

BvhTree::~BvhTree()
{
    for (AABBox* node : nodes_)
    {
        delete node;
    }

    root_ = nullptr;
}


void BvhTree::BuildRootBox_()
{
    std::list<AABBox*> all_children;

    for (AABBox* node : nodes_)
    {
        all_children.push_back(node);
    }

    root_ = new AABContainer(std::move(all_children), nullptr);
    nodes_.push_back(root_);
}


void BvhTree::SplitIntoBoxes_(AABContainer* cur_cont)
{
    RLSU_ASSERT(cur_cont);
    RLSU_DUMP(Dump());
    ASSERT_HANDLE(Assert());

    if (cur_cont->GetChildrenNum() <= box_max_capa_)
    {
        return;
    }

    Math::Vector3 diagonal = cur_cont->GetP1() - cur_cont->GetP0();
    double max_side = std::max({diagonal.GetX(), diagonal.GetY(), diagonal.GetZ()});


    std::function<int(Primitives::Point3, Primitives::Point3)> CompareAxis;
    Primitives::Point3 check_point = cur_cont->GetP1();

    enum changing_axisT { X, Y, Z } changing_axis;

    if (Math::DoubleEq(max_side, diagonal.GetX()))
    {
        check_point.SetX(cur_cont->GetP0().GetX() + max_side / 2);

        CompareAxis = Primitives::Point3::CompareX;
        changing_axis = changing_axisT::X;
    }

    else if (Math::DoubleEq(max_side, diagonal.GetY()))
    {
        check_point.SetY(cur_cont->GetP0().GetY() + max_side / 2);

        CompareAxis = Primitives::Point3::CompareY;
        changing_axis = changing_axisT::Y;
    }

    else if (Math::DoubleEq(max_side, diagonal.GetZ()))
    {
        check_point.SetZ(cur_cont->GetP0().GetZ() + max_side / 2);

        CompareAxis = Primitives::Point3::CompareZ;
        changing_axis = changing_axisT::Z;
    }

    else
        RLSU_ASSERT(false, "error in defining CompareAxis");

    RLSU_DUMP(check_point.Dump("check_point!"));

    AABContainer* sub_cont = new AABContainer();
    
    for (auto child_it = cur_cont->GetChildren().begin(); child_it != cur_cont->GetChildren().end(); /*-*/ )
    {
        const AABBox* const child = *child_it;

        // transfer from cur to sub if the element lies in (mb cross) sub
        if (CompareAxis(child->GetP1(), check_point) > 0)
        {
            auto child_it_cpy = child_it++;
            
            ERROR_HANDLE(sub_cont->MoveChildFromOtherContainer(child_it_cpy, cur_cont));

            continue;
        }

        child_it++;
    }

    switch(changing_axis)
    {
    case X : cur_cont->SetP1_X(sub_cont->GetP0().GetX() + Math::CmpEps);  break;
    case Y : cur_cont->SetP1_Y(sub_cont->GetP0().GetY() + Math::CmpEps);  break;
    case Z : cur_cont->SetP1_Z(sub_cont->GetP0().GetZ() + Math::CmpEps);  break;

    default: RLSU_ERROR("invalid changing_axis = {}", changing_axis);
    }

    if (sub_cont->IsEmpty())
    {
        cur_cont->SetP1(check_point);
        SplitIntoBoxes_(cur_cont);

        delete sub_cont;
        return;
    }

    AABContainer *new_father = new AABContainer();
    nodes_.push_back(new_father);
    
    nodes_.push_back(sub_cont);
    
    if (cur_cont != root_)
    {
        AABContainer* cur_cont_father = cur_cont->father;
        RLSU_ASSERT(cur_cont_father);

        ERROR_HANDLE(cur_cont_father->AbandonChild(cur_cont));
        ERROR_HANDLE(cur_cont_father->AddChild(new_father));
    }
    
    else
        root_ = new_father;


    ERROR_HANDLE(new_father->AddChild(cur_cont));
    ERROR_HANDLE(new_father->AddChild(sub_cont));

    ERROR_HANDLE(SplitIntoBoxes_(cur_cont));
    ERROR_HANDLE(SplitIntoBoxes_(sub_cont));

    if (cur_cont->IsDegraded())
        ResolveDegradedContainer_(cur_cont);

    if (sub_cont->IsDegraded())
        ResolveDegradedContainer_(sub_cont);

    if (cur_cont->IsEmpty())
        ResolveEmptyContainer_(cur_cont);

    if (sub_cont->IsEmpty())
        ResolveEmptyContainer_(sub_cont);
}


void BvhTree::ResolveDegradedContainer_(AABContainer* degraded_cont)
{
    RLSU_ASSERT(degraded_cont->IsDegraded());

    degraded_cont->father->AddChild(degraded_cont->GetChildren().front());
    degraded_cont->father->AbandonChild(degraded_cont);

    EraseNode_(degraded_cont);
}

void BvhTree::ResolveEmptyContainer_(AABContainer* empty_cont)
{
    RLSU_ASSERT(empty_cont->IsEmpty());  

    empty_cont->father->AbandonChild(empty_cont);

    EraseNode_(empty_cont);
}

void BvhTree::EraseNode_(const AABContainer* erasing_node)
{
    for (auto iterator = nodes_.begin(); iterator != nodes_.end(); iterator++)
    {
        if (*iterator == erasing_node)
        {
            AABBox* node = *iterator;
            delete node;

            nodes_.erase(iterator);
            return;
        }
    }
}


} // namespace Geometry::MathEngine