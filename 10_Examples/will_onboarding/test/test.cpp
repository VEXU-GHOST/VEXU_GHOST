/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "will_onboarding/pub_sub.hpp"
#include "gtest/gtest.h"

class PubNodeTest : public ::testing::Test
{
protected:
  PubNodeTest()
  {
    pub_node_ = std::make_shared<will_onboarding::PubNode>();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  std::shared_ptr<will_onboarding::PubNode> pub_node_;
};

TEST_F(PubNodeTest, CreateHiMsgReturnsExpectedString) {
  EXPECT_EQ(pub_node_->create_hi_msg(), "hi");
}

TEST_F(PubNodeTest, CreateByeMsgReturnsExpectedString) {
  EXPECT_EQ(pub_node_->create_bye_msg(), "bye");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
