#include "try.h"

MyClass::MyClass()
: a(10), b(20) {}

int MyClass::geta() const {return a;}
int MyClass::getb() const {return b;}
int MyClass::add() const {return a + b;}
