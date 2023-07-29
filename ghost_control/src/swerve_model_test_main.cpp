
#include <iostream>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"
#include "casadi/casadi.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "urdf_parser/urdf_parser.h"
#include "urdf_model/link.h"

#include <ghost_util/dfs_search.hpp>

using ghost_util::DFSSearch;
using ghost_util::SearchNode;

class URDFLinkSearchNode : public SearchNode<urdf::Link>
{
public:
    URDFLinkSearchNode(std::shared_ptr<urdf::Link> node) : SearchNode<urdf::Link>(node)
    {
        m_id = node->name;
    }

    std::vector<std::shared_ptr<ghost_util::SearchNodeBase>> getChildren() override
    {
        std::vector<std::shared_ptr<SearchNodeBase>> child_list{};
        for (auto &child : m_node->child_links)
        {
            child_list.push_back(std::make_shared<URDFLinkSearchNode>(child));
        }
        return child_list;
    };

    void processNode(int depth) override
    {
        for (int i = 0; i < depth; i++)
        {
            std::cout << "\t";
        }
        std::cout << m_node->name << std::endl;
        // link_names.push_back(link_ptr->name);
    };
};

int main(int argc, char *argv[])
{

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ghost_description");
    std::string urdf_path = package_share_directory + "/urdf/ghost1.urdf";
    std::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDFFile(urdf_path);

    // Traverse URDF Tree
    std::shared_ptr<urdf::Link> base_link_ptr;
    urdf->getLink("base_link", base_link_ptr);

    DFSSearch(std::make_shared<URDFLinkSearchNode>(base_link_ptr));

    // dfs(base_link, visited, depth, print_link);

    // for(const auto & link : links){
    // std::cout << "Name: " << link->name << std::endl;
    // std::cout << "\tParent Joint: " << link->parent_joint << std::endl;
    // std::cout << "\tInertial:" << std::endl;
    // std::cout << "\t\tPose: " << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.position.x << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.position.y << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.position.z << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.rotation.x << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.rotation.y << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.rotation.z << std::endl;
    // std::cout << "\t\t\t" << link->inertial->origin.rotation.w << std::endl;
    // std::cout << "\t\tMass: " << link->inertial->mass << std::endl;
    // std::cout << "\t\tInertial: " << link->inertial->ixx << ", " << link->inertial->ixy << ", " << link->inertial->ixz << ", " << link->inertial->iyy << ", " << link->inertial->iyz << ", " << link->inertial->izz << std::endl;

    // std::cout << "Child Joints:" << std::endl;
    // for(const auto& joint : link->child_joints){
    //     std::cout << "\t\t" << joint->name << std::endl;
    // }
    // std::cout << "Child Links:" << std::endl;
    // for(const auto& link_name : link->child_links){
    //     std::cout << "\t\t" << link_name << std::endl;
    // }

    // std::cout << std::endl;
    // }
    // model.initFile(urdf_path);

    return 0;
}