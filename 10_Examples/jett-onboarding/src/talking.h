#ifndef TALKING_H_
#define TALKING_H_

#include <iostream>
#include "string.h"

using namespace std;

class talking
{
public:
  int num1;

  talking();    // constructor
  ~talking();   // destructor

  string talk(string words);
  int numberPrint();

  int twoToPower(int i);
};

#endif
