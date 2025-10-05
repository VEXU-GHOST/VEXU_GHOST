#include <iostream>
#include "hello_world.h"
using namespace std;

int main(int argc, char **argv) {
    std::cout << "Hello World!" << std::endl;
    hello_world myHelloWorld;
    myHelloWorld.myMethod(2);
    return 0;
}