#pragma once
#include <iostream>

class TrivialMethodsClass {
public:
	TrivialMethodsClass() {
		std::cout << "test constructor" << std::endl;
	}
	int test_method(int i) {
		return i * i;
	}
};