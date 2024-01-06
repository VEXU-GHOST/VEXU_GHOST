#pragma once

namespace ghost_util {

bool getRandomBool(){
	return (bool) (rand() % 2);
}

float getRandomFloat(){
	return (float) rand() + 0.0001 * rand();
}

int getRandomInt(){
	return (int) getRandomFloat();
}

}