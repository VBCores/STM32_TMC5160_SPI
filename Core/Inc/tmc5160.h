/*
 * tmc5160.h
 *
 *  Created on: Aug 11, 2023
 *      Author: VR
 */

#ifndef INC_TMC5160_H_
#define INC_TMC5160_H_

//#include "stm32g4xx_hal.h"
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

struct motor_config
{
	int8_t motor_type;
	uint32_t full_steps;
	float gear_ratio;
	float upper_limit_effort;
	float max_effort_by_default;
	int8_t max_irun_scaler;
};

typedef struct motor_config motor_config;


void tmc5160_init();

void tmc5160_position(int32_t position);

void tmc5160_move(int32_t vel);

void tmc5160_acceleration(uint32_t acc);

int32_t tmc5160_position_read();

void tmc5160_velocity(uint32_t vel);

void tmc5160_set_default_vel();

int32_t sign_extend_bits_to_32(int32_t x, uint8_t bits);

int32_t tmc5160_velocity_read();

void tmc5160_effort(double, double);

void tmc5160_set_motor_direction(int8_t);

void tmc5160_set_zero();

void tmc5160_arm();

void tmc5160_disarm();

void tmc5160_stop();

void tmc5160_write(uint8_t* data);

void tmc5160_read(uint8_t* WData, uint8_t* RData);

void clamp_value(double *min_value, double *value, double *max_value);

double clamp_value_noref(double min_value, double value, double max_value);

uint8_t tmc5160_torque_to_curent(double effort, double max_effort);

void tmc5160_motor_config(int8_t motor_type, int8_t direction, uint32_t full_steps, float gear_ratio, float upper_limit_effort, motor_config * mc);

#endif /* INC_TMC5160_H_ */
