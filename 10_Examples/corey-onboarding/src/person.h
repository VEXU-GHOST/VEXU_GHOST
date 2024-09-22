#ifndef PERSON_H
#define PERSON_H

#include <string>
#include <iostream>



class Person {

private:
	std::string name;
	int age;

	
public:
	Person(std::string person_name, int person_age);

	std::string getName();
	int getAge();

	void printInfo();

};

#endif
