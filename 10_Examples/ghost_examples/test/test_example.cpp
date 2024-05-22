#include "ghost_examples/ros_subscriber_example.hpp"
#include "gtest/gtest.h"

class TestExample : public ::testing::Test
{
protected:
  TestExample()
  {
    subscriber_node_ = std::make_shared<ghost_examples::ROSSubscriberExample>();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
  std::shared_ptr<ghost_examples::ROSSubscriberExample> subscriber_node_;
};

TEST_F(TestExample, testAddInts) {
  // A few ways to test the results of your code
  EXPECT_EQ(subscriber_node_->add_ints(2, 3), 5);
  EXPECT_TRUE(subscriber_node_->add_ints(2, 3) == 5);
  EXPECT_FALSE(subscriber_node_->add_ints(2, 3) == 4);
}

TEST_F(TestExample, testAddFloats) {
  // Sometimes floats are a tiny tiny bit off and dont evaluate equal because of how computers store them.
  // Thus, it is usually safer to use a different equality check.
  EXPECT_FLOAT_EQ(subscriber_node_->add_floats(2, 3), 5.0);
}

TEST_F(TestExample, testStrings) {
  // Remember that strings need a special equality check.
  // ("test" == "test") may not be true!
  EXPECT_STREQ("test", "test");
}

TEST_F(TestExample, testThrow) {
  // EXPECT_THROW(subscriber_node_->function_that_throws_error());
  // EXPECT_NO_THROW(subscriber_node_->do_nothing());

  try {
    subscriber_node_->function_that_throws_error();
  } catch (const std::runtime_error & e) {
    std::cout << "We caught the error: " << e.what() << std::endl;
  }
}

int main(int argc, char ** argv)
{
  // This line is always required if you are going to instantiate a ROS node.
  // For testing regular C++ libraries, it should be excluded.
  rclcpp::init(argc, argv);

  // Standard GTest main
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
