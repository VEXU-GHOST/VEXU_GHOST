#pragma once

#include <math.h>

namespace ghost_util {

bool getRandomBool(){
	return (bool) (rand() % 2);
}

float getRandomFloat(){
	return (float) rand() + 0.0001 * rand();
}

float getRandomFloat(float max){
	return std::fmod((float) rand() + 0.0001 * rand(), max);
}

float getRandomDouble(){
	return (double) rand() + 0.0001 * rand();
}

float getRandomDouble(float max){
	return std::fmod((double) rand() + 0.0001 * rand(), max);
}

int getRandomInt(){
	return (int) getRandomFloat();
}

}