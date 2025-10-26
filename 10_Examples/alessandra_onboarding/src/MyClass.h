#ifndef MYCLASS_H
#define MYCLASS_H

#include <string>

class MyClass {

private:
    int val;
    std::string name;

public:
    
    MyClass(int v = 0, const std::string& n = "empty")
        : val(v), name(n) {}

    void addValue(int add2 = 2) {
        val += add2;}

    void changeName(const std::string& difname) {
        name = difname;}

    int getValue() const{
       return val;
    }

    const std::string& getName() const {
        return name;
    }
    
};
#endif