/*
 * tmc5160.c
 *
 *  Created on: Aug 11, 2023
 *      Author: VR
 */


#include "tmc5160.h"
#include "spi.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#if (USE_FREERTOS == 1)
#include "cmsis_os.h"
#define tmc5160_delay(x)   osDelay(x)
#else
#define tmc5160_delay(x)   HAL_Delay(x)
#endif




void tmc5160_position(int32_t position)
{
	uint8_t WData[5] = {0};

	WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; //SPI send: 0xA000000000; // RAMPMODE = 1 (position move)
	  tmc5160_write(WData);

	WData[0] = 0xAD; //moving register
	WData[1] = (position & 0xFF000000) >> 24; //position in steps
	WData[2] = (position & 0x00FF0000) >> 16;
	WData[3] = (position & 0x0000FF00) >> 8;
	WData[4] = (position & 0x000000FF);
	tmc5160_write(WData);
}

void tmc5160_move(int32_t vel)
{
	vel *= 1.3981013; //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet
	int32_t v1;
	uint8_t WData[5] = {0};

	v1 = vel >> 1; // >> 1 (to divide by 2)

	if (vel < 0) //select positive or negative mode depending on vel sign
	{
		  WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x02; //SPI send: 0xA000000001; // RAMPMODE = 1 (positive velocity move)
		  tmc5160_write(WData);
	}
	else
	{
		  WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x01; //SPI send: 0xA000000001; // RAMPMODE = 2 (negative velocity move)
		  tmc5160_write(WData);
	}

	//Acceleration threshold velocity V1
	WData[0] = 0xA5; //V1 speed register
	WData[1] = (v1 & 0xFF000000) >> 24;
	WData[2] = (v1 & 0x00FF0000) >> 16;
	WData[3] = (v1 & 0x0000FF00) >> 8;
	WData[4] = (v1 & 0x000000FF);
	tmc5160_write(WData);

	vel = abs(vel);
	//sending VMAX
	WData[0] = 0xA7; //VMAX speed register
	WData[1] = (vel & 0xFF000000) >> 24;
	WData[2] = (vel & 0x00FF0000) >> 16;
	WData[3] = (vel & 0x0000FF00) >> 8;
	WData[4] = (vel & 0x000000FF);
	tmc5160_write(WData);
}

void tmc5160_velocity(uint32_t vel)
{
	vel *= 1.3981013; //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet
	uint32_t v1;
	uint8_t WData[5] = {0};

	v1 = vel >> 1; // >> 1 (to divide by 2)

	//Acceleration threshold velocity V1
	WData[0] = 0xA5; //V1 speed register
	WData[1] = (v1 & 0xFF000000) >> 24;
	WData[2] = (v1 & 0x00FF0000) >> 16;
	WData[3] = (v1 & 0x0000FF00) >> 8;
	WData[4] = (v1 & 0x000000FF);
	tmc5160_write(WData);

	//VMAX
	WData[0] = 0xA7; //VMAX speed register
	WData[1] = (vel & 0xFF000000) >> 24;
	WData[2] = (vel & 0x00FF0000) >> 16;
	WData[3] = (vel & 0x0000FF00) >> 8;
	WData[4] = (vel & 0x000000FF);
	tmc5160_write(WData);

}

void tmc5160_effort(double effort)
{
double clamp_effort;
uint8_t IRUN = 0;
uint8_t IHOLD = 0;
uint8_t WData[5] = {0};

clamp_effort = clamp_value_noref(0.0, effort, MOTOR_MAX_TORQUE);
IRUN = torque_to_curent(clamp_effort, MOTOR_MAX_TORQUE);
IHOLD = IRUN >> 1;

WData[0] = 0x90;
WData[1] = 0x00;
WData[2] = 0x00; // IHOLDDELAY=0 TODO?
WData[3] = IRUN; // IRUN
WData[4] = IHOLD; // IHOLD

tmc5160_write(WData);
}


void tmc5160_acceleration(uint32_t acc)
{
	uint8_t WData[5] = {0};
	uint8_t Tk = 65; //time constant. Acceleration equals steps/Tk see ref on p. 81 of datasheet
	acc = acc / Tk;

	WData[0] = 0xA3; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // Start acceleration = 10 (Near start)
	tmc5160_write(WData);

	// A1 First acceleration
	WData[0] = 0xA4;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	// AMAX Acceleration above V1
	WData[0] = 0xA6;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	// DMAX Deceleration above V1
	WData[0] = 0xA8;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	// D1 Deceleration below V1
	WData[0] = 0xAA;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	WData[0] = 0xAB; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // VSTOP = 10 Stop velocity (Near to zero)
	tmc5160_write(WData);
}


void tmc5160_write(uint8_t* data)
{
	HAL_GPIO_WritePin(_STEPPER_MOTOR_DRIVER_NSS_GPIO, _STEPPER_MOTOR_DRIVER_NSS_PIN, GPIO_PIN_RESET); //CS LOW
	HAL_SPI_Transmit(&_STEPPER_MOTOR_DRIVER_SPI, data, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_STEPPER_MOTOR_DRIVER_NSS_GPIO, _STEPPER_MOTOR_DRIVER_NSS_PIN, GPIO_PIN_SET); //CS HIGH
}


void tmc5160_read(uint8_t* WData, uint8_t* RData)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_SPI_TransmitReceive(&_STEPPER_MOTOR_DRIVER_SPI, WData, RData, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_SPI_TransmitReceive(&_STEPPER_MOTOR_DRIVER_SPI, WData, RData, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
}


int32_t tmc5160_position_read()
{
	uint8_t WData[5] = {0};
	uint8_t RData[5] = {0};
	WData[0] = 0x21; //XACTUAL register address
	tmc5160_read(WData, RData);

	int32_t response = 0;

    response |= (RData[1]);
    response <<= 8;
    response |= (RData[2]);
    response <<= 8;
    response |= (RData[3]);
    response <<= 8;
    response |= (RData[4]);

	return response;
}

int32_t tmc5160_velocity_read()
{
	uint8_t WData[5] = {0};
	uint8_t RData[5] = {0};
	WData[0] = 0x22; //VACTUAL register address
	tmc5160_read(WData, RData);

	int32_t response = 0;

    response |= (RData[1] & 0xFF);
    response <<= 8;
    response |= (RData[2] & 0xFF);
    response <<= 8;
    response |= (RData[3] & 0xFF);
    response <<= 8;
    response |= (RData[4] & 0xFF);

    int32_t rv = 0;
    rv = sign_extend_bits_to_32(response, 24);

	return (rv / 1.3981013); //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet
}

void tmc5160_init()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //DRV SLEEP 0 for power on, 1 for power off
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //SPI_MODE ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); //SD_MODE OFF INTERNAL RAMP GENERATOR ON

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //DIR
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //STEP
	HAL_Delay(100);

	uint8_t WData[5] = {0};

	WData[0] = 0xEC; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0xC3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
	tmc5160_write(WData);

	WData[0] = 0x90; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x01; WData[4] = 0x01; //  IHOLDDELAY=10,  IRUN=10/31,  IHOLD=02/31
	tmc5160_write(WData);

	WData[0] = 0x91; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // TPOWERDOWN=10: Delay before power down in stand still
	tmc5160_write(WData);

	WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x04; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	tmc5160_write(WData);

	tmc5160_velocity(1000000); //initial vel config

	WData[0] = 0x93; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0xC8; // TPWM_THRS=200 yields a switching velocity about 35000 = ca. 30RPM
	tmc5160_write(WData);

	WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; //SPI send: 0xA000000000; // RAMPMODE = 0 (Target position move)
	tmc5160_write(WData);

	WData[0] = 0xA3; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // Start acceleration = 10 (Near start)
	tmc5160_write(WData);

	WData[0] = 0xA4; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x6e; WData[4] = 0x20; // A1 = 10 000 First acceleration
	tmc5160_write(WData);

	WData[0] = 0xA6; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x23; WData[4] = 0x88; // AMAX = 5 000 Acceleration above V1
	tmc5160_write(WData);

	WData[0] = 0xA8; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x23; WData[4] = 0x88; // DMAX = 5 000 Deceleration above V1
	tmc5160_write(WData);

	WData[0] = 0xAA; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x6e; WData[4] = 0x20; // D1 = 10 000 Deceleration below V1
	tmc5160_write(WData);

	WData[0] = 0xAB; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // VSTOP = 10 Stop velocity (Near to zero)
	tmc5160_write(WData);

	HAL_Delay(100);
}


void tmc5160_set_inverse_motor_direction()
{
	  uint8_t WData[5] = {0};
	  WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x14; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	  tmc5160_write(WData);
}

void tmc5160_set_forward_motor_direction()
{
	  uint8_t WData[5] = {0};
	  WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x04; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	  tmc5160_write(WData);
}

void tmc5160_set_zero()
{
	uint8_t WData[5] = {0};
	uint32_t pos = 0;
	tmc5160_stop();
	pos = tmc5160_position_read();
	WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x03; // RAMPMODE = 3 (HOLD mode)
	tmc5160_write(WData);

	WData[0] = 0xA1; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; // Set zero
	tmc5160_write(WData);
}



void tmc5160_disarm()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); //DRV SLEEP 0 for power on, 1 for power off
}

void tmc5160_arm()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //DRV SLEEP 0 for power on, 1 for power off
}

void tmc5160_stop()
{
	uint8_t WData[5] = {0};
	uint32_t pos = 0;

	WData[0] = 0xA3; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; // Start acceleration = 10 (Near start)
	tmc5160_write(WData);

	//VMAX
	WData[0] = 0xA7; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00;
	tmc5160_write(WData);

	pos = tmc5160_position_read();
	tmc5160_position(pos);
}


//TODO make sure that is works for less than 24 bit values
int32_t sign_extend_bits_to_32(int32_t x, uint8_t bits) {

	uint32_t sign_mask = 0;
	//getting value of sign bit
	sign_mask = 1u << (bits - 1);
	uint32_t sign_bit = 0;
	sign_bit = x & sign_mask;
	if(sign_bit) //if value < 0 therefore sign_bit == 1, fill first 8 bits with 1
	{
		int32_t res = 0;
		int32_t mask = 0b11111111;
		res |= x;
		res |= (mask << (bits));
		return res;
	}
    return x; //else return value itself
}

double clamp_value_noref(double min_value, double value, double max_value)
{
	value = (((min_value < value)? value : min_value) > max_value)? max_value: value;
	return value;
}

void clamp_value(double *min_value, double *value, double *max_value)
{
	*value = (((*min_value < *value)? *value : *min_value) > *max_value)? *max_value: *value;
}

uint8_t torque_to_curent(double effort, double max_effort)
{
	uint8_t IRUN = 0;
	IRUN = (effort / max_effort) * MOTOR_MAX_IRUN_SCALER;
	return IRUN;
}

