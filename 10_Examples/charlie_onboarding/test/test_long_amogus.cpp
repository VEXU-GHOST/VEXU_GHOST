#include <string>

#include "charlie_onboarding/long_amogus.hpp"
#include "gtest/gtest.h"

TEST(LongAmogusTest, AddsTwoTorsoRowsEachCall)
{
  LongAmogus amogus;
  const std::string segment =
    "\n⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀";
  const std::string first = amogus.next();
  const std::string second = amogus.next();
  const std::string third = amogus.next();
  const auto legs = first.find("\n⠀⠀⠛⢿⣿⣿⣿⣿⣇");
  ASSERT_NE(legs, std::string::npos);
  EXPECT_EQ(second, first.substr(0, legs) + segment + segment + first.substr(legs));
  EXPECT_EQ(third, first.substr(0, legs) + segment + segment + segment + segment +
    first.substr(legs));
}

TEST(LongAmogusTest, InstancesGrowIndependently)
{
  LongAmogus first;
  LongAmogus second;
  const auto initial = first.next();
  first.next();
  EXPECT_EQ(second.next(), initial);
}
