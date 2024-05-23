/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

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
    for (auto & child_node_ptr : child_list) {
      if (m_visited.count(child_node_ptr->getID()) == 0) {
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
