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

#include <memory>
#include <string>
#include <vector>

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

template<typename T>
class SearchNode : public SearchNodeBase
{
public:
  SearchNode(std::shared_ptr<T> node)
  : m_node(node)
  {
  }

protected:
  std::shared_ptr<T> m_node;
};

} // namespace ghost_util
