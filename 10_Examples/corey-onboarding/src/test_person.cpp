#include "person.h"
#include <cassert>
#include <iostream>

int main()
{

	Person person1("Bob", 25);

	assert(person1.getName() == "Bob");
	assert(person1.getAge() == 25);

	std::cout << "All tests passed!" << std::endl;

	return 0;
}
