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

#if (_ENCODER_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define tmc5160_delay(x)   osDelay(x)
#else
#define tmc5160_delay(x)   HAL_Delay(x)
#endif


void tmc5160_write(uint8_t* data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_Delay(1);
	HAL_SPI_Transmit(&_STEPPER_MOTOR_DRIVER_SPI, data, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
	HAL_Delay(1);
}

void tmc5160_ReadWrite(uint8_t* WData, uint8_t* RData)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&_STEPPER_MOTOR_DRIVER_SPI, WData, RData, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
	HAL_Delay(1);
}



void tmc5160_write_IT(uint8_t* data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_Delay(1);
	HAL_SPI_Transmit_IT(&_STEPPER_MOTOR_DRIVER_SPI, data, 5);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
	HAL_Delay(1);
}

void tmc5160_ReadWrite_IT(uint8_t* WData, uint8_t* RData)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_Delay(1);
	HAL_SPI_TransmitReceive_IT(&_STEPPER_MOTOR_DRIVER_SPI, WData, RData, 5);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
	HAL_Delay(1);
}





