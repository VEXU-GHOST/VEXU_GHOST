#include <iostream>
#include "firstCPPClass.h"

using namespace std;

firstCPPClass::firstCPPClass() : name("null"){};
firstCPPClass::firstCPPClass(string n) : name(n){};


string firstCPPClass::getName() {
    return name;
}


/*int main() {
    firstCPPClass helloUser("Nuzhat\n");
    cout << helloUser.getName();

    return 0;
}*/