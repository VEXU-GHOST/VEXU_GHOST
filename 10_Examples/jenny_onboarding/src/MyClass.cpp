// MyClass.cpp
#include "MyClass.h"

MyClass::MyClass() : value(22) {}

int MyClass::getValue() const {
    return value;
}

void MyClass::setValue(int newValue) {
    value = newValue;
}
