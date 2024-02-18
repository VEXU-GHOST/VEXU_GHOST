#include <iostream>

#include "annie-onboarding/hello_world.hpp"

namespace annie_onboarding {

        demoClass::demoClass(std::string n, int a, std::string m){
            name = n;
            age = a;
            breed = m;
        }

        void demoClass::setName(std::string n){
            name = n;
        }

        void demoClass::setAge(int a){
            age = a;
        }

        void demoClass::setBreed(std::string m){
            breed = m;
        }

        void demoClass::displayInfo(){
            std::cout << "Name: " << name << std::endl;
            std::cout<< "Age: " << age << std::endl;
            std::cout<< "Breed: " << breed << std::endl;
        }

int add_ints(int a, int b){

    int sum = a + b;
    return sum; 

}


float add_floats(float a, float b){

    float sum = a + b;
    return sum;

}

float function_that_throws_error(){
    return 1/0;
}

bool comparing_strings(std::string a, std::string b){
    
    bool verify = false;

    if(a == b){
        verify = true;
    }

    return verify;

}

}