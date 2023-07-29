#pragma once

#include <memory>
#include "search_node.hpp"

namespace ghost_util
{
    class DFSSearch
    {
    public:
        DFSSearch(std::shared_ptr<SearchNodeBase> root_ptr)
        {
            m_depth = 0;
            m_root_ptr = root_ptr;

            root_ptr->processNode(m_depth);
            search(root_ptr);
        }

    private:
        void search(std::shared_ptr<SearchNodeBase> node_ptr)
        {
            auto child_list = node_ptr->getChildren();
            for (auto &child_node_ptr : child_list)
            {
                if (m_visited.count(child_node_ptr->getID()) == 0)
                {
                    m_visited[child_node_ptr->getID()] = true;
                    m_depth++;
                    child_node_ptr->processNode(m_depth);
                    search(child_node_ptr);
                    m_depth--;
                }
            }
        }
        std::shared_ptr<SearchNodeBase> m_root_ptr;
        std::unordered_map<std::string, bool> m_visited;
        int m_depth;
    };
} // namespace ghost_util