/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"


uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t status = 0;
	uint8_t buffer[4] = {0, 0, 0, 0};

	status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 4);
	if (!status) {
		*value = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
	}
	else {
		printf("[VL53L4CD] RdDWord write phase failed: reg=0x%02X\n", RegisterAdress);
	}

	return status;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t status = 0;
	uint8_t buffer[2] = {0, 0};

	status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 2);
	if (!status) {
		*value = (buffer[0] << 8) + buffer[1];
	}
	else {
		printf("[VL53L4CD] RdWord write phase failed: reg=0x%02X\n", RegisterAdress);
	}

	return status;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t status = 0;
	status = VL53L4CD_I2CRead(dev, RegisterAdress, value, 1);
	if (status) {
		printf("[VL53L4CD] RdByte write phase failed: reg=0x%02X\n", RegisterAdress);
	}
	return status;

}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t status = 0;
	status = VL53L4CD_I2CWrite(dev, RegisterAdress, &value, 1);
	if (status) {
		printf("[VL53L4CD] WrByte write phase failed: reg=0x%02X val=0x%02X\n", RegisterAdress, value);
	}
	return status;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t status = 0;
	uint8_t buffer[2];

	buffer[0] = value >> 8;
	buffer[1] = value & 0x00FF;
	status = VL53L4CD_I2CWrite(dev, RegisterAdress, (uint8_t *)buffer, 2);
	if (status) {
		printf("[VL53L4CD] WrWord write phase failed: reg=0x%02X val=0x%04X\n", RegisterAdress, value);
	}
	return status;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t status = 0;
	uint8_t buffer[4];

	buffer[0] = (value >> 24) & 0xFF;
	buffer[1] = (value >> 16) & 0xFF;
	buffer[2] = (value >>  8) & 0xFF;
	buffer[3] = (value >>  0) & 0xFF;
	status = VL53L4CD_I2CWrite(dev, RegisterAdress, (uint8_t *)buffer, 4);
	if (status) {
		printf("[VL53L4CD] WrDWord write phase failed: reg=0x%02X val=0x%08X\n", RegisterAdress, value);
	}
	return status;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
	uint8_t status = 255;
	Delay(TimeMs);
	return status;
}


