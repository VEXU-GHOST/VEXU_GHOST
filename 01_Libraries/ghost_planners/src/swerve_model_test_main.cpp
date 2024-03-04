#include <iostream>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"
#include "casadi/casadi.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "urdf_model/link.h"
#include "urdf_parser/urdf_parser.h"

#include <ghost_common/graph_search/dfs_search.hpp>

using ghost_common::DFSSearch;
using ghost_common::SearchNode;
using urdf::Joint;

class URDFLinkSearchNode : public SearchNode<urdf::Link> {
public:
	URDFLinkSearchNode(std::shared_ptr<urdf::Link> node,
	                   std::shared_ptr<std::vector<std::string> > &names_ptr) :
		SearchNode<urdf::Link>(node){
		m_id = node->name;
		m_names_ptr = names_ptr;
		m_names_ptr->push_back(m_id);
	}

	std::vector<std::shared_ptr<ghost_common::SearchNodeBase> > getChildren() override {
		std::vector<std::shared_ptr<SearchNodeBase> > child_list{};
		for(auto &child : m_node->child_links){
			child_list.push_back(std::make_shared<URDFLinkSearchNode>(child, m_names_ptr));
		}
		return child_list;
	}

	void processNode(int depth) override {
		auto link = m_node;
		auto joint = m_node->parent_joint;
		if(joint != NULL){
			if(joint->type == Joint::CONTINUOUS){
				std::cout << std::endl;
				std::cout << "Link Name: " << link->name << std::endl;
				std::cout << "Mass: " << link->inertial->mass << std::endl;
				std::cout << "Inertia: " << link->inertial->izz << std::endl;

				std::cout << "Joint Name: " << joint->name << std::endl;
				std::cout << "Friction: " << joint->dynamics->friction << std::endl;
				std::cout << "Damping: " << joint->dynamics->damping << std::endl;

				// std::cout << "\t\t\t" << link->inertial->origin.position.x << std::endl;
				// std::cout << "\t\t\t" << link->inertial->origin.position.y << std::endl;
				// std::cout << "\t\t\t" << link->inertial->origin.position.z << std::endl;
				// std::cout << "\t\t\t" << link->inertial->origin.rotation.x << std::endl;
				// std::cout << "\t\t\t" << link->inertial->origin.rotation.y << std::endl;
				// std::cout << "\t\t\t" << link->inertial->origin.rotation.z << std::endl;
				// std::cout << "\t\t\t" << link->inertial->origin.rotation.w << std::endl;
			}
		}

		// link_names.push_back(link_ptr->name);
	}

private:
	std::shared_ptr<std::vector<std::string> > m_names_ptr;
};

int main(int argc, char *argv[]){
	std::string package_share_directory = ament_index_cpp::get_package_share_directory("ghost_description");
	std::string urdf_path = package_share_directory + "/urdf/ghost1.urdf";
	std::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDFFile(urdf_path);

	// Traverse URDF Tree
	std::shared_ptr<urdf::Link> base_link_ptr;
	urdf->getLink("base_link", base_link_ptr);

	std::cout << base_link_ptr->name << std::endl;
	std::cout << base_link_ptr->inertial->mass << std::endl;
	std::cout << base_link_ptr->inertial->izz << std::endl;

	auto names = std::make_shared<std::vector<std::string> >();
	DFSSearch(std::make_shared<URDFLinkSearchNode>(base_link_ptr, names));

	return 0;
}