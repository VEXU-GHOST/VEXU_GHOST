#include <iostream>
#include "talking.h"
#include "string.h"

int test()
{

    talking t1;

    // talk function
    string words1 = "Hello World";
    t1.talk(words1);
    t1.talk("Hello Again");

    // number print
    t1.num1 = 0;
    t1.numberPrint();
    t1.num1 += 10;
    t1.numberPrint();

    // twoToPower
    for (int i = 0; i < 10; i++)
    {
        cout << "2^" << i << " = " << t1.twoToPower(i) << endl;
    }

    return 0;
}