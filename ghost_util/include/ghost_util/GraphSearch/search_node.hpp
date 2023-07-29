#include <vector>
#include <string>
#include <memory>

namespace ghost_util
{
    class SearchNodeBase
    {
    public:
        SearchNodeBase() = default;
        std::vector<std::shared_ptr<SearchNodeBase>> virtual getChildren() = 0;
        void virtual processNode(int depth) = 0;
        std::string getID()
        {
            return m_id;
        }

    protected:
        std::string m_id;
    };

    template <typename T>
    class SearchNode : public SearchNodeBase
    {
    public:
        SearchNode(std::shared_ptr<T> node) : m_node(node) {}

    protected:
        std::shared_ptr<T> m_node;
    };
} // namespace ghost_util