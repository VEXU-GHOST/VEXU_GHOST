#pragma once

#include <memory>

/*
template <typename TYPE, typename T>
bool MyFunction(TYPE obj, T TYPE::*mp)
//                          ^^^^^^^^^
{
    if ((obj.*mp) == 5)
//          ^^^^
        return false;

    // ... <== DON'T FORGET TO RETURN SOMETHING IN THIS CASE,
    //         OTHERWISE YOU WILL GET UNDEFINED BEHAVIOR
}

*/
namespace ghost_util
{
    template <typename node>
    class DFSSearch
    {
    public:
        DFSSearch(std::shared_ptr<node> root_ptr, void (*node_function)(std::shared_ptr<node>, int))
        {
            m_depth = 0;
            m_root_ptr = root_ptr;
            m_node_function = node_function;

            search(root_ptr);
        }

    private:
        void search(std::shared_ptr<node> node_ptr)
        {
            for (auto &child_node_ptr : node_ptr->child_links)
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
        std::shared_ptr<node> m_root_ptr;
        void (*m_node_function)(std::shared_ptr<node>, int);

        std::unordered_map<std::shared_ptr<node>, bool> m_visited;
        int m_depth;
    };
} // namespace ghost_util