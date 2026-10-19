#include <gtest/gtest.h>
#include "amy_onboarding/calculator.hpp"

TEST(CalculatorTest, Addition)
{
  Calculator calculator;
  EXPECT_EQ(calculator.add(2, 3), 5);
  EXPECT_EQ(calculator.add(-1, 1), 0);
}

TEST(CalculatorTest, Multiplication)
{
  Calculator calculator;
  EXPECT_EQ(calculator.multiply(4, 5), 20);
  EXPECT_EQ(calculator.multiply(-2, 3), -6);
}
