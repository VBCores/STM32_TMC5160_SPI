/*
 * tmc5160.h
 *
 *  Created on: Aug 11, 2023
 *      Author: VR
 */

#ifndef INC_TMC5160_H_
#define INC_TMC5160_H_

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>



#define MOTOR NEMA14


#define		_STEPPER_MOTOR_DRIVER_SPI			hspi1
#define		_STEPPER_MOTOR_DRIVER_USE_FREERTOS		1  /* FreeRTOS by default */
#define     _STEPPER_MOTOR_DRIVER_NSS_GPIO GPIOA
#define     _STEPPER_MOTOR_DRIVER_NSS_PIN GPIO_PIN_4
#define nop() asm volatile("nop")


#define BYTE(value, n)    (((value) >> ((n) << 3)) & 0xFF)

/**
  * @brief  Read driver register into given var
  * @param  given variable by ref
  * @param  timeout Timeout duration
  * @retval bool status
  */

void tmc5160_init();

void tmc5160_position(int32_t position);

int32_t tmc5160_position_read();

void tmc5160_velocity(uint32_t vel);

int32_t sign_extend_bits_to_32(int32_t x, uint8_t bits);

int32_t tmc5160_velocity_read();

void tmc5160_effort(double effort);

void tmc5160_write(uint8_t* data);

void tmc5160_read(uint8_t* WData, uint8_t* RData);

void clamp_value(double *min_value, double *value, double *max_value);

double clamp_value_noref(double min_value, double value, double max_value);

uint8_t torque_to_curent(double effort, double max_effort);




#define NEMA14_MAX_IRUN_SCALER 10
#define NEMA17_MAX_IRUN_SCALER 12
#define NEMA23_MAX_IRUN_SCALER 31

//TODO to research global scaler adjust for different types of motor to achieve full range of current control. For now just limits the IRUN multiplier.
#if MOTOR == NEMA14
	#define MOTOR_MAX_IRUN_SCALER NEMA14_MAX_IRUN_SCALER
#elif MOTOR == NEMA17
	#define MOTOR_MAX_IRUN_SCALER NEMA17_MAX_IRUN_SCALER
#elif MOTOR == NEMA23
	#define MOTOR_MAX_IRUN_SCALER NEMA23_MAX_IRUN_SCALER
#endif

//MAX HOLDING TORQUE * gear ratio / 5 (which is empirical decrease for moving torque)
#define NEMA14_MAX_TORQUE 0.5
#define NEMA17_MAX_TORQUE 3.9
#define NEMA23_MAX_TORQUE 10.2

#if MOTOR == NEMA14
	#define MOTOR_MAX_TORQUE NEMA14_MAX_TORQUE
#elif MOTOR == NEMA17
	#define MOTOR_MAX_TORQUE NEMA17_MAX_TORQUE
#elif MOTOR == NEMA23
	#define MOTOR_MAX_TORQUE NEMA23_MAX_TORQUE
#endif

// 200 (fullsteps) * 256 (microsteps) * Gear ratio
#define NEMA14_FULLSTEPS    983204
#define NEMA17_FULLSTEPS	2560000
#define NEMA23_FULLSTEPS    2560000

#if MOTOR == NEMA14
	#define MOTOR_FULLSTEP NEMA14_FULLSTEPS
#elif MOTOR == NEMA17
	#define MOTOR_FULLSTEP NEMA17_FULLSTEPS
#elif MOTOR == NEMA23
	#define MOTOR_FULLSTEP NEMA23_FULLSTEPS
#endif

// Gear ratio
#define NEMA14_GR 19 //TODO possible to correct ratio to 19.38/187 for more precise velocity calculation
#define NEMA17_GR 50
#define NEMA23_GR 50

#if MOTOR == NEMA14
	#define MOTOR_GR NEMA14_GR
#elif MOTOR == NEMA17
	#define MOTOR_GR NEMA17_GR
#elif MOTOR == NEMA23
	#define MOTOR_GR NEMA23_GR
#endif


#endif /* INC_TMC5160_H_ */
