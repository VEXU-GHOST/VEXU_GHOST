#include <iostream>
#include "amy_onboarding/calculator.hpp"

int main()
{
  Calculator calculator;

  std::cout << "Amy's onboarding program!" << std::endl;
  std::cout << "2 + 3 = " << calculator.add(2, 3) << std::endl;
  std::cout << "4 * 5 = " << calculator.multiply(4, 5) << std::endl;

  return 0;
}
