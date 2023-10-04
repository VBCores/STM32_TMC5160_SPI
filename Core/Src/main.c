/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <tmc5160.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*
 * Limits for w  (10<w<100)
 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //Config via pins see p.116 of TMC5160 datasheet


  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //DRV SLEEP 0 for power on, 1 for power off
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //SPI_MODE ON
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); //SD_MODE OFF INTERNAL RAMP GENERATOR ON

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //DIR
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //STEP
  HAL_Delay(100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

/*
  SPI send: 0xEC000100C3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
  SPI send: 0x9000061F0A; // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
  SPI send: 0x910000000A; // TPOWERDOWN=10: Delay before power down in stand still
  SPI send: 0x8000000004; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
  SPI send: 0x93000001F4; // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
*/



  uint8_t RData[5] = {0};
  uint8_t WData[5] = {0};


  WData[0] = 0xEC; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0xC3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0x90; WData[1] = 0x00; WData[2] = 0x06; WData[3] = 0x02; WData[4] = 0x01; // IHOLD_IRUN: IHOLD=10,  IRUN=31 (max. current), IHOLDDELAY=10
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0x91; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // TPOWERDOWN=10: Delay before power down in stand still
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x01; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0x93; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0xC8; // TPWM_THRS=200 yields a switching velocity about 35000 = ca. 30RPM
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xA4; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x4e; WData[4] = 0x20; //SPI send: 0xA4000003E8; // A1 = 20 000 First acceleration
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xA6; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x13; WData[4] = 0x88; //SPI send: 0xA6000001F4; // AMAX = 5 000 Acceleration above V1
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xA5; WData[1] = 0x00; WData[2] = 0x16; WData[3] = 0xe3; WData[4] = 0x60; //SPI send: 0xA50000C350; // V1 = 1 200 000 Acceleration threshold velocity V1
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xA7; WData[1] = 0x00; WData[2] = 0x16; WData[3] = 0x20; WData[4] = 0x10; //SPI send: 0xA700030D40; // VMAX = 1 500 000
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xA8; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x13; WData[4] = 0x88; //SPI send: 0xA8000002BC; // DMAX = 5 000 Deceleration above V1
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xAA; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x4e; WData[4] = 0x20; //SPI send: 0xAA00000578; // D1 = 10 000 Deceleration below V1
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xAB; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; //SPI send: 0xAB0000000A; // VSTOP = 10 Stop velocity (Near to zero)
  tmc5160_ReadWrite(WData, RData);

  WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; //SPI send: 0xA000000000; // RAMPMODE = 0 (Target position move)
  tmc5160_ReadWrite(WData, RData);

  HAL_Delay(100);

  while (1)
  {



	  WData[0] = 0xAD; WData[1] = 0x00; WData[2] = 0x0e; WData[3] = 0xc9; WData[4] = 0x28  ; // SPI send: 0xADFFFF3800; // XTARGET = -60000 (Move one rotation left (200*256 microsteps)
	  tmc5160_ReadWrite(WData, RData);

	  HAL_Delay(5000);

	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);


	  WData[0] = 0xAD; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; // SPI send: 0xADFFFF3800; // XTARGET = 60000 (Move one rotation left (200*256 microsteps)
	  tmc5160_ReadWrite(WData, RData);

	  HAL_Delay(5000);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
