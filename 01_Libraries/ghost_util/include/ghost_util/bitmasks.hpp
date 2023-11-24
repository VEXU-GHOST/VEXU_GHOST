#pragma once

#include <stdexcept>
#include <vector>
#include <stdint.h>

namespace ghost_util {

extern const uint32_t BITMASK_ARR_32BIT[32];

void setBit(unsigned char& byte, int bit_num, bool val){
	if(bit_num >= 8){
		throw std::runtime_error("[ghost_util::setBit] Error: bit_num must be between 0 and 7.");
	}
	if(val){
		byte |= BITMASK_ARR_32BIT[bit_num];
	}
	else{
		byte &= ~BITMASK_ARR_32BIT[bit_num];
	}
}

bool getBit(unsigned char byte, int bit_num){
	if(bit_num >= 8){
		throw std::runtime_error("[ghost_util::setBit] Error: bit_num must be between 0 and 7.");
	}
	return byte & BITMASK_ARR_32BIT[bit_num];
}

unsigned char packByte(const std::vector<bool>& bool_arr, bool big_endian = true){
	if(bool_arr.size() != 8){
		throw std::runtime_error("[ghost_util::packByte] Error: bool array must be of size 8.");
	}
	unsigned char byte = 0;
	for(int i = 0; i < 8; i++){
		if(big_endian){
			setBit(byte, i, bool_arr[i]);
		}
		else{
			setBit(byte, 7 - i, bool_arr[i]);
		}
	}
	return byte;
}

} // namespace ghost_util