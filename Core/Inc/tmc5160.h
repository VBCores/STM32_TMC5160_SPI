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
#include <tmc5160_constants.h>

#include <tmc5160_registers.h>


#define		_STEPPER_MOTOR_DRIVER_SPI			hspi1
#define		_STEPPER_MOTOR_DRIVER_USE_FREERTOS		0  /* NOT FreeRTOS by default */
#define     _STEPPER_MOTOR_DRIVER_NSS_GPIO GPIOA
#define     _STEPPER_MOTOR_DRIVER_NSS_PIN GPIO_PIN_4
#define nop() asm volatile("nop")
#define WRITE_ACCESS 0x80


#define BYTE(value, n)    (((value) >> ((n) << 3)) & 0xFF)

/**
  * @brief  Read driver register into given var
  * @param  given variable by ref
  * @param  timeout Timeout duration
  * @retval bool status
  */

void tmc5160_init();

void tmc5160_position(int32_t position);

void tmc5160_velocity(uint32_t vel);

void tmc5160_effort(uint8_t effort);

void tmc5160_write(uint8_t* data);

void clamp_value(double *min_value, double *value, double *max_value);

double clamp_value_noref(double min_value, double value, double max_value);

uint8_t torque_to_curent(double effort, double max_effort);

//void tmc5160_ReadWrite(uint8_t* Tdata, uint8_t* Rdata);
//void tmc5160_write_IT(uint8_t* data);
//void tmc5160_ReadWrite_IT(uint8_t* WData, uint8_t* RData);


#endif /* INC_TMC5160_H_ */
