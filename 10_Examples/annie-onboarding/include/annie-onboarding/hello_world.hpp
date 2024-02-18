#pragma once

#include <string>

namespace annie_onboarding{
class demoClass{
    public:

        std::string name;
        int age;
        std::string breed;

        demoClass(std::string n, int a, std::string m);

        void setName(std::string n);

        void setAge(int a);

        void setBreed(std::string m);

        void displayInfo();

};

int add_ints(int a, int b);

float add_floats(float a, float b);

float function_that_throws_error();

bool comparing_strings(std::string a, std::string b);
}