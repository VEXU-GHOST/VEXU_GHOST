#include "Person.hpp"

Person::Person(std::string person_name, int person_age)
: name(person_name), age(person_age) {}

std::string Person::getName()
{
  return name;
}

int Person::getAge()
{
  return age;
}

void Person::printInfo()
{
  std::cout << "Name: " << name << "Age: " << std::endl;

}

int main()
{
  // Create an object of the Person class

//   Person person1 = new Person("Alice", 30);
  std::shared_ptr<Person> person1 = std::make_shared<Person>("Alice", 30);

  // Print information about the person
  person1->printInfo();    // Output: Name: Alice, Age: 30

  return 0;
}
