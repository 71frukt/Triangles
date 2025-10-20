#include "Geometry/common/geometry_obj.hpp"
#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/math_engine/collision_handler.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/error_handler.hpp"

#include "RLogSU/logger.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
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
    
    for (auto child_it = cur_cont->children.begin(); child_it != cur_cont->children.end(); /*-*/ )
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
    case X : cur_cont->SetP1_X(check_point.GetX() + Math::CmpEps);  break;
    case Y : cur_cont->SetP1_Y(check_point.GetY() + Math::CmpEps);  break;
    case Z : cur_cont->SetP1_Z(check_point.GetZ() + Math::CmpEps);  break;

    default: RLSU_ERROR("invalid changing_axis = {}", changing_axis);
    }
    

    if (cur_cont->IsEmpty())
    {
        // if (cur_cont != root_)
        // {
        //     nodes_.push_back(sub_cont);
        //     cur_cont->father->AddChild(sub_cont);
        //     ERROR_HANDLE(cur_cont->father->AbandonChild(cur_cont));
        // }

        // else
        // {
        //     nodes_.push_back(sub_cont);
        //     root_ = sub_cont;
        // }

        // RLSU_DUMP(Dump());

        // EraseNode_(cur_cont);
        // RLSU_DUMP(Dump());

        // return;

        cur_cont->children = sub_cont->children;
        cur_cont->SetP0(sub_cont->GetP0());
        cur_cont->SetP1(sub_cont->GetP1());

        delete sub_cont;
        return;
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

    if (std::find(nodes_.begin(), nodes_.end(), cur_cont) != nodes_.end() && cur_cont->IsDegraded())
        ERROR_HANDLE(ResolveDegradedContainer_(cur_cont));

    if (std::find(nodes_.begin(), nodes_.end(), cur_cont) != nodes_.end() && cur_cont->IsEmpty())
        ERROR_HANDLE(ResolveEmptyContainer_(cur_cont));
}


void BvhTree::ResolveDegradedContainer_(AABContainer* degraded_cont)
{
    RLSU_ASSERT(degraded_cont->IsDegraded());

    degraded_cont->father->AddChild(degraded_cont->children.front());
    ERROR_HANDLE(degraded_cont->father->AbandonChild(degraded_cont));

    EraseNode_(degraded_cont);
}

void BvhTree::ResolveEmptyContainer_(AABContainer* empty_cont)
{
    RLSU_ASSERT(empty_cont->IsEmpty());  

    if (empty_cont != root_)
        ERROR_HANDLE(empty_cont->father->AbandonChild(empty_cont));

    EraseNode_(empty_cont);
}

void BvhTree::EraseNode_(const AABBox* erasing_node)
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


std::vector<bool> BvhTree::GetIntersections() const
{
    size_t res_table_max_sz = nodes_.size();

    std::vector<bool> intersections(res_table_max_sz);

    for (const AABBox* node : nodes_)
    {
        if (typeid(*node) == typeid(AABLeaf))
        {
            const AABLeaf& leaf = static_cast<const AABLeaf&>(*node);
            GetLeafIntersectionInContainer_(leaf, *root_, intersections);
        }
    }

    return intersections;
}

void BvhTree::GetLeafIntersectionInContainer_(const AABLeaf& leaf, const AABContainer& cont, std::vector<bool>& intersections) const
{
    ASSERT_HANDLE(leaf.Assert());
    ASSERT_HANDLE(cont.Assert());

    leaf.Dump();
    cont.Dump();

    if (! (Math::DoubleGE(leaf.GetP0().GetX(), cont.GetP1().GetX()) || Math::DoubleGE(leaf.GetP0().GetY(), cont.GetP1().GetY()) || Math::DoubleGE(leaf.GetP0().GetZ(), cont.GetP1().GetZ())
        || Math::DoubleBE(leaf.GetP1().GetX(), cont.GetP0().GetX()) || Math::DoubleBE(leaf.GetP1().GetY(), cont.GetP0().GetY()) || Math::DoubleBE(leaf.GetP1().GetZ(), cont.GetP0().GetZ())))
    {
        for (const AABBox* child : cont.children)
        {
            if (child == &leaf)
                continue;

            if (typeid(*child) == typeid(AABLeaf))
            {
                const AABLeaf& other_leaf = static_cast<const AABLeaf&>(*child);

                Geometry::MathEngine::CollisionCodeT coll_code = ERROR_HANDLE(Geometry::MathEngine::Interact(*leaf.inscribed, *other_leaf.inscribed)->CollisionCode());
            
                if (coll_code != Geometry::MathEngine::NOTHING)
                {
                    intersections.at(leaf      .GetId()) = true;                    
                    intersections.at(other_leaf.GetId()) = true;
                }
            }

            else
            {
                const AABContainer& other_cont = static_cast<const AABContainer&>(*child);
                GetLeafIntersectionInContainer_(leaf, other_cont, intersections);
            }
        }
    }
}

// void BvhTree::GetIntersectionsInContainer_(std::vector<std::vector<bool>>& res_table, const AABContainer* cont) const
// {
//     RLSU_ASSERT(cont);
//     RLSU_ASSERT(typeid(*cont) == typeid(AABContainer));

//     const auto& children = cont->children;
//     size_t children_num  = cont->children.size();

//     for (auto it_1 = children.begin(); it_1 != children.end(); it_1++)
//     {
//         const AABBox* child_1 = *it_1;

//         if (typeid(*child_1) == typeid(AABContainer))
//             GetIntersectionsInContainer_(res_table, static_cast<const AABContainer*>(child_1));

//         if (typeid(*child_1) != typeid(AABLeaf))
//             continue;

//         const AABLeaf* leaf_1 = static_cast<const AABLeaf*>(child_1);

//         for (auto it_2 = std::next(it_1); it_2 != children.end(); it_2++)
//         {
//             const AABBox* child_2 = *it_2;
//             if (typeid(*child_2) != typeid(AABLeaf))
//                 continue;

//             const AABLeaf* leaf_2 = static_cast<const AABLeaf*>(child_2);

//             Geometry::MathEngine::CollisionCodeT coll_code = ERROR_HANDLE(Geometry::MathEngine::Interact(*leaf_1->inscribed, *leaf_2->inscribed)->CollisionCode());
            
//             if (coll_code != Geometry::MathEngine::NOTHING)
//             {
//                 res_table.at(leaf_1->GetId()).at(leaf_2->GetId()) = true;
//                 res_table.at(leaf_2->GetId()).at(leaf_1->GetId()) = true;                
//             }
//         }
//     }
// }

} // namespace Geometry::MathEngine