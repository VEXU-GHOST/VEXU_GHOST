#include <iostream>
#include "Johnny_Shen_onboarding/hello_world.hpp"




int main(int argc, char **argv) {
    std::cout << "Hello World!" << std::endl;
    Add myObj;
    std::cout << myObj.add(100,200);
    return 0;
}
