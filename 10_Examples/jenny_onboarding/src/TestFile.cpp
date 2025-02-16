#include <iostream>
#include "MyClass.h"

void testDefaultConstructor() {
    MyClass obj;
    if (obj.getValue() == 22) {
        std::cout << "Test Passed: Default constructor works as expected." << std::endl;
    } else {
        std::cout << "Test Failed: Default constructor does not set value to 22." << std::endl;
    }
}

void testSetAndGetValue() {
    MyClass obj;
    obj.setValue(45);
    if (obj.getValue() == 45) {
        std::cout << "Test Passed: setValue and getValue work as expected." << std::endl;
    } else {
        std::cout << "Test Failed: setValue and getValue do not work correctly." << std::endl;
    }
}

int main() {
    testDefaultConstructor();
    testSetAndGetValue();

    return 0;
}
