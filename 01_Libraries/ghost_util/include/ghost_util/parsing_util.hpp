#pragma once

#include <sstream>
#include <string>
#include <vector>

#include <iostream>

namespace ghost_util {

template <typename T>
T convertFromString(std::string val){
	return (T) val;
}

template <>
int convertFromString(std::string val){
	return stoi(val);
}

template <>
float convertFromString(std::string val){
	return std::stof(val);
}

template <>
double convertFromString(std::string val){
	return std::stod(val);
}

template <typename T>
std::vector<T> getVectorFromString(const std::string& input, const char delim){
	// construct a stream from the string
	std::stringstream ss(input);
	std::vector<T> data;
	std::string s;
	while(std::getline(ss, s, delim)){
		data.push_back(convertFromString<T>(s));
	}
	return data;
}

} // namespace ghost_util