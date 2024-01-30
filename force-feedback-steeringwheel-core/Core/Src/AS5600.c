/*
 * AS5600.c
 *
 *  Created on: Jan 30, 2024
 *      Author: beunprogrammeur
 */
#include <stdint.h>
#include "AS5600.h"
#include "i2c.h"

uint16_t receive_angle();

const uint8_t AS5600_DEVICE_ADDRESS = 0x6C;
const uint8_t AS5600_REGISTER_STATUS = 0x0B;
const uint8_t AS5600_REGISTER_RAW_ANGLE_HI = 0x0C;
const uint8_t AS5600_REGISTER_RAW_ANGLE_LO = 0x0D;
const uint16_t precision = 4096; // the AS5600 encoder has a precision of 4096 counts
const uint16_t half_precision = precision / 2;

struct AS5600_data_t {
	uint16_t encoder_angle;
	uint16_t old_encoder_angle;
	uint16_t home_angle;
	int32_t system_angle;
	int32_t old_system_angle;
} ;

static struct AS5600_data_t data;

void set_angle(uint16_t angle)
{
	data.encoder_angle = angle;
	data.old_encoder_angle = angle;
	data.home_angle = angle;
	data.system_angle = angle;
	data.old_system_angle = angle;
}

void AS5600_init(I2C_HandleTypeDef *hi2c)
{
	set_angle(0);
	data.encoder_angle = receive_angle();
}

void AS5600_set_home()
{
	set_angle(data.encoder_angle);
}


int32_t AS5600_get_angle()
{
	data.encoder_angle = receive_angle();
	int16_t delta = data.encoder_angle - data.old_encoder_angle;

	// compensate for the bit of the encoder where we step from 0 to 360 and vice versa
	if (delta > half_precision) delta -= precision;
	else if (delta < -half_precision) delta += precision;

	data.old_encoder_angle = data.encoder_angle;
	data.old_system_angle = data.system_angle;
	data.system_angle += delta;

	return data.system_angle - data.home_angle;
}

uint16_t receive_angle()
{
	uint8_t buffer[2];
	buffer[0] = AS5600_REGISTER_RAW_ANGLE_HI;

	HAL_I2C_Master_Transmit(&hi2c1, AS5600_DEVICE_ADDRESS, buffer, 1, 3);
	HAL_I2C_Master_Receive(&hi2c1,  AS5600_DEVICE_ADDRESS, buffer, 2, 3);

	return (buffer[0] << 8) | buffer[1];
}
