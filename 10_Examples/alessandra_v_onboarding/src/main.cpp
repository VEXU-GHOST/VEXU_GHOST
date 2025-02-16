#include <iostream>
#include "try.h"

int main()
{
  MyClass obj;
  std::cout << "This is a: " << obj.geta() << ", This is b: " << obj.getb() << std::endl;
  std::cout << "Sum: " << obj.add() << std::endl;
  return 0;
}
