#include <iostream>

#include "charlie_onboarding/duck.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

int main()
{
#ifdef _WIN32
  SetConsoleOutputCP(CP_UTF8);
#endif

  const Duck duck;
  std::cout << duck.art() << '\n' << duck.quack() << std::endl;
  return 0;
}
