#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/vector3.hpp"
#include "Geometry/primitives/primitives.hpp"
#include "Geometry/math_engine/bvh_tree.hpp"
#include "Geometry/math_engine/aabb.hpp"
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <functional>

namespace Geometry::MathEngine {

BvhTree::BvhTree(const std::vector<const GeomObj*>& objects, const size_t box_max_capa_)
    : box_max_capa_(box_max_capa_)
    , root_(nullptr)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        ASSERT_HANDLE(objects[i]->Assert());

        nodes_.emplace_back(std::make_unique<AABLeaf>(objects[i]));
    }

    ERROR_HANDLE(BuildRootBox_());
    ERROR_HANDLE(SplitIntoBoxes_(*root_));
}


void BvhTree::BuildRootBox_()
{
    std::list<const AABBox*> all_children;

    for (size_t i = 0; i < nodes_.size(); i++)
    {
        all_children.push_back(nodes_[i].get());
    }

    auto root = ERROR_HANDLE(std::make_unique<AABContainer>(std::move(all_children), nullptr));
    root->father = (const AABBox*) &root;   // корень сам себе отец
    nodes_.push_back(std::move(root));

    root_ = root.get();
}


void BvhTree::SplitIntoBoxes_(AABBox& cur_box)
{
    ASSERT_HANDLE(cur_box.Assert());
    
    if (typeid(cur_box) != typeid(AABContainer))
        return;

    AABContainer& cur_container = static_cast<AABContainer&>(cur_box);
    std::list<const AABBox*>& cur_children = cur_container.children;

    if (cur_children.size() < box_max_capa_)
        return;


    // this is the part that we will separate from this current container. 
    // it will be to the right/higher/further along the x/y/z axis, respectively
    std::list<const AABBox*> sub_children;

    Math::Vector3 diagonal = cur_box.GetP1() - cur_box.GetP0();
    double max_side = std::max({diagonal.GetX(), diagonal.GetY(), diagonal.GetZ()});


    std::function<int(Primitives::Point3, Primitives::Point3)> CompareAxis;
    Primitives::Point3 check_point = cur_container.GetP0();

    enum changing_axisT { X, Y, Z } changing_axis;

    if (Math::DoubleEq(max_side, diagonal.GetX()))
    {
        check_point.SetX(cur_container.GetP0().GetX() + max_side / 2);

        CompareAxis = Primitives::Point3::CompareX;
        changing_axis = changing_axisT::X;
    }

    else if (Math::DoubleEq(max_side, diagonal.GetY()))
    {
        check_point.SetY(cur_container.GetP0().GetY() + max_side / 2);

        CompareAxis = Primitives::Point3::CompareY;
        changing_axis = changing_axisT::Y;
    }

    else if (Math::DoubleEq(max_side, diagonal.GetZ()))
    {
        check_point.SetZ(cur_container.GetP0().GetZ() + max_side / 2);

        CompareAxis = Primitives::Point3::CompareZ;
        changing_axis = changing_axisT::Z;
    }


    for (auto child_it = cur_children.begin(); child_it != cur_children.end();  )
    {
        const AABBox* const child = *child_it;

        // transfer from cur to sub if the element lies in (mb cross) sub
        if (CompareAxis(child->GetP1(), check_point) > 0)
        {
            auto child_it_cpy = child_it++;
            sub_children.splice(sub_children.end(), cur_children, child_it_cpy);
            continue;
        }

        child_it++;
    }


    std::unique_ptr<AABContainer> sub_container = std::make_unique<AABContainer>(std::move(sub_children), cur_box.father);
    AABContainer* sub_container_ptr = sub_container.get();    

    switch(changing_axis)
    {
    case X : cur_container.SetP1_X(sub_container->GetP0().GetX() + Math::CmpEps);  break;
    case Y : cur_container.SetP1_Y(sub_container->GetP0().GetY() + Math::CmpEps);  break;
    case Z : cur_container.SetP1_Z(sub_container->GetP0().GetZ() + Math::CmpEps);  break;

    default: RLSU_ERROR("invalid changing_axis = {}", changing_axis);
    }

    
    std::unique_ptr<AABContainer> new_father = std::make_unique<AABContainer>(cur_box.father);

    cur_container. father = new_father.get();
    sub_container->father = new_father.get();

    new_father->children.push_back(&cur_container);
    new_father->children.push_back(sub_container_ptr);
    
    nodes_.push_back(std::move(new_father));
    nodes_.push_back(std::move(sub_container));

    SplitIntoBoxes_( cur_container);
    SplitIntoBoxes_(*sub_container_ptr);
}



}