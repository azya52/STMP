/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define FLASH_START_ADDR	0x08000000
#define FLASH_FIRM_SIZE		0x00004400					//16kB for firmware
#define FLASH_FLASH_SIZE	0x00020000					//undocumented 128kB flash
#define FLASH_MSD_SIZE		(FLASH_FLASH_SIZE-FLASH_FIRM_SIZE)	//Mass storage size
#define FLASH_MSD_START_ADDR	(FLASH_START_ADDR + FLASH_FIRM_SIZE)	//USB MSD start address

#define PORT_CLK GPIOB
#define PIN_CLK GPIO_PIN_7
#define PIN_CLK_NUM 7

#define PORT_DATA GPIOB
#define PIN_DATA GPIO_PIN_8
#define PIN_DATA_NUM 8

#define PORT_SELECT GPIOB
#define PIN_SELECT GPIO_PIN_9
#define PIN_SELECT_NUM 9

#define PORT_LED GPIOC
#define PIN_LED GPIO_PIN_13
#define PIN_LED_NUM 13

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
