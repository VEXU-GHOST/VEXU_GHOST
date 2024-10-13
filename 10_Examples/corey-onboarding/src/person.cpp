#include "person.h"

Person::Person(std::string person_name, int person_age) : name(person_name), age(person_age) {}

std::string Person::getName() {
	return name;
}

int Person::getAge() {
	return age;
}

void Person::printInfo() {
	std::cout << "Name: " << name << "Age: " << std::endl;

}
