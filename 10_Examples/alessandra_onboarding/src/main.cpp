#include <iostream>
#include <string>
#include "MyClass.h"

int main() {
    
    MyClass my_obj(5, "test");

    std::cout << "before" << std::endl;
    std::cout << "value: " << my_obj.getValue() << std::endl;
    std::cout << "name: " << my_obj.getName() << std::endl;

    my_obj.addValue(); 
    my_obj.changeName("test success");

    std::cout << "after" << std::endl;
    std::cout << "value: " << my_obj.getValue() << std::endl;
    std::cout << "name:" << my_obj.getName() << std::endl;

    return 0;
}