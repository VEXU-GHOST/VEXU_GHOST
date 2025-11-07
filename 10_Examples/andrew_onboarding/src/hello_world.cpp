#include <iostream>
#include <string>
#include "trivial_methods.cpp"

int main(int argc, char **argv) {
    std::cout << "Hello World!" << std::endl;
    trivial_methods triv = trivial_methods();
    std::cout << triv.isEven(2) << std::endl;
    
    std::string uhh = "very slay";
    std::string umm = "sfonfg";

    triv.testHowSlay(uhh);
    triv.testHowSlay(umm);



    return 0;
}