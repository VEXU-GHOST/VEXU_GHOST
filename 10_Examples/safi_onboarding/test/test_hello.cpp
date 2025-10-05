#include <gtest/gtest.h>
#include "sayHello.hpp"

TEST(helloTest, HelloWorks) {
  sayHello testHello;
  EXPECT_EQ(testHello.greet(), "Hello from sayHello!");
}

TEST(helloTest, AddTest) {
  sayHello testHello;
  EXPECT_EQ(testHello.add(2, 3), 5);
}
