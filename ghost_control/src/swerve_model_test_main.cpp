
#include <iostream>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"
#include "casadi/casadi.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "urdf_parser/urdf_parser.h"

#include <ghost_util/dfs_search.hpp>

using ghost_util::DFSSearch;

int main(int argc, char *argv[])
{

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ghost_description");
    std::string urdf_path = package_share_directory + "/urdf/ghost1.urdf";
    std::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDFFile(urdf_path);

    // Traverse URDF Tree
    auto base_link = urdf->getLink("base_link");
    std::cout << base_link->name << std::endl;

    std::unordered_map<std::shared_ptr<const urdf::Link>, bool> visited;
    int depth = 0;

    auto print_link = [](std::shared_ptr<const urdf::Link> link_ptr, int depth)
    {
        for (int i = 0; i < depth; i++)
        {
            std::cout << "\t";
        }
        std::cout << link_ptr->name << std::endl;
    };

    DFSSearch<const urdf::Link>(base_link, print_link);
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