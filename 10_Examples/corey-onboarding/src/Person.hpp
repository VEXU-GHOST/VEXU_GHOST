#ifndef PERSON_H
#define PERSON_H

#include <string>
#include <iostream>
#include <memory>

class Person
{
public:
  Person(std::string person_name, int person_age);

  std::string getName();
  int getAge();

  void printInfo();

private:
  std::string name;
  int age;

};

#endif
