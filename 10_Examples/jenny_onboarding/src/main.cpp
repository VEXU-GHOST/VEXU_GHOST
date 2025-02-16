// main.cpp
#include <iostream>
#include "MyClass.h"

using namespace std;

int main() {
    MyClass obj;

    cout << "Default value: " << obj.getValue() << endl;

    obj.setValue(45);

    cout << "Updated value: " << obj.getValue() << endl;

    return 0;
}
