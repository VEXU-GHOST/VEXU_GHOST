#include "annie-onboarding/hello_world.hpp"
#include <iostream>

using annie_onboarding::demoClass;

int main() {
    std::cout <<"Testing if this builds in the first place" << std::endl;
    std::cout << "Hello World!" << std::endl;
    std::cout << "slay" << std::endl;


    demoClass cat1 = demoClass("meowmeow", 3, "orange cat");
    cat1.displayInfo();

    cat1.setName("barkbark");
    cat1.setAge(12);
    cat1.setBreed("not orange cat");

    cat1.displayInfo();
    
    
    return 0;
}