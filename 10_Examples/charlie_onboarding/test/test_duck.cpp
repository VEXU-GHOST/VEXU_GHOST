#include <string>

#include "charlie_onboarding/duck.hpp"
#include "gtest/gtest.h"

TEST(DuckTest, ReturnsArtwork)
{
  const Duck duck;
  EXPECT_FALSE(std::string(duck.art()).empty());
}

TEST(DuckTest, Quacks)
{
  const Duck duck;
  EXPECT_STREQ(duck.quack(), "Quack!");
}
