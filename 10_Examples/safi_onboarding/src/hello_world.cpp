#include <iostream>
#include "sayHello.hpp"

int main() {
    sayHello testHello;
    std::cout << testHello.greet() << std::endl;
    std::cout << testHello.add(1,5) << std::endl;
    return 0;
}
