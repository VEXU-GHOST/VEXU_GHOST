#ifndef TALKING_H_
#define TALKING_H_

#include <iostream>
#include "string.h"

using namespace std;

class talking
{
public:
    int num1;

    talking();  // constructor
    ~talking(); // destructor

    void talk(string words);
    void numberPrint();

    int twoToPower(int i);
};

#endif