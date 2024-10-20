#include "person.h"
#include <gtest/gtest.h>
#include <iostream>
#include "ghost_examples/ros_subscriber_example.hpp"
#include "gtest/gtest.h"

TEST_F(Person, Constructor) {

  Person person1("Bob", 25);

  EXPECT_EQ(person1.getName(), "Bob");
  EXPECT_EQ(person1.getAGe(), "25");

}

int main(int argc, char ** argv)
{

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

}
