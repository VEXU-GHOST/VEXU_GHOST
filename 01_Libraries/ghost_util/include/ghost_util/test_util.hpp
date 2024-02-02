#pragma once

#include <math.h>

namespace ghost_util {

bool getRandomBool(){
	return (bool) (rand() % 2);
}

float getRandomFloat(){
	float sign = (getRandomBool()) ? 1 : -1;
	return ((float) rand() + 0.0001 * rand()) * sign;
}

float getRandomFloat(float max){
	float sign = (getRandomBool()) ? 1 : -1;
	return sign * std::fmod((float) rand() + 0.0001 * rand(), max);
}

float getRandomDouble(){
	float sign = (getRandomBool()) ? 1 : -1;
	return sign * ((double) rand() + 0.0001 * rand());
}

float getRandomDouble(float max){
	float sign = (getRandomBool()) ? 1 : -1;
	return sign * std::fmod((double) rand() + 0.0001 * rand(), max);
}

int getRandomInt(){
	return (int) getRandomFloat();
}

}