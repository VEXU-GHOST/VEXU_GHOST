#include "talking.h"
#include <iostream>
#include "string.h"

// using namespace std;

talking::talking()
{
    this->num1 = 10;
}

talking::~talking()
{
}

void talking::talk(string words)
{
    cout << words << endl;
}

void talking::numberPrint()
{
    cout << num1 << endl;
}

int talking::twoToPower(int i)
{
    int value = 1;
    while (i > 0)
    {
        value *= 2;
        i--;
    }
    return value;
}