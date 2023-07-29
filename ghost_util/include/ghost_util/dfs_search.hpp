#pragma once

#include <memory>

namespace ghost_util
{
    template <typename node_t>
    class DFSSearch
    {
    public:
        DFSSearch(
            std::shared_ptr<node_t> root_ptr,
            void (*node_function)(std::shared_ptr<node_t>, int),
            std::vector<std::shared_ptr<node_t>> (*get_children_function)(std::shared_ptr<node_t> node_ptr))
        {
            m_depth = 0;
            m_root_ptr = root_ptr;
            m_node_function = node_function;
            m_get_children_function = get_children_function;

            search(root_ptr);
        }

    private:
        void search(std::shared_ptr<node_t> node_ptr)
        {
            auto child_list = m_get_children_function(node_ptr);
            for (auto &child_node_ptr : child_list)
            {
                if (m_visited.count(child_node_ptr) == 0)
                {
                    m_visited[child_node_ptr] = true;
                    m_depth++;
                    m_node_function(child_node_ptr, m_depth);
                    search(child_node_ptr);
                    m_depth--;
                }
            }
        }
        std::shared_ptr<node_t> m_root_ptr;
        void (*m_node_function)(std::shared_ptr<node_t>, int);
        std::vector<std::shared_ptr<node_t>> (*m_get_children_function)(std::shared_ptr<node_t> node_ptr);

        std::unordered_map<std::shared_ptr<node_t>, bool> m_visited;
        int m_depth;
    };
} // namespace ghost_util