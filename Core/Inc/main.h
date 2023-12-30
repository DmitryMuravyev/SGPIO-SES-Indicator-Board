/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define SLOAD_SDA_Pin GPIO_PIN_0
#define SLOAD_SDA_GPIO_Port GPIOF
#define SCLOCK_SCL_Pin GPIO_PIN_1
#define SCLOCK_SCL_GPIO_Port GPIOF
#define SDATAIN_ADDRESS_Pin GPIO_PIN_0
#define SDATAIN_ADDRESS_GPIO_Port GPIOA
#define DRIVE3_Pin GPIO_PIN_1
#define DRIVE3_GPIO_Port GPIOA
#define DRIVE2_Pin GPIO_PIN_2
#define DRIVE2_GPIO_Port GPIOA
#define DRIVE1_Pin GPIO_PIN_3
#define DRIVE1_GPIO_Port GPIOA
#define DRIVE0_Pin GPIO_PIN_4
#define DRIVE0_GPIO_Port GPIOA
#define ALARM_Pin GPIO_PIN_5
#define ALARM_GPIO_Port GPIOA
#define ACTIVITY_Pin GPIO_PIN_6
#define ACTIVITY_GPIO_Port GPIOA
#define IDENTIFY_Pin GPIO_PIN_7
#define IDENTIFY_GPIO_Port GPIOA
#define ADDRESS_SELECT_Pin GPIO_PIN_1
#define ADDRESS_SELECT_GPIO_Port GPIOB
#define CONTROLLER_TYPE_Pin GPIO_PIN_9
#define CONTROLLER_TYPE_GPIO_Port GPIOA
#define BPTYPE_SELECT_Pin GPIO_PIN_10
#define BPTYPE_SELECT_GPIO_Port GPIOA
#define BACKPLANE_TYPE_Pin GPIO_PIN_13
#define BACKPLANE_TYPE_GPIO_Port GPIOA
#define SDATAOUT_RESET_Pin GPIO_PIN_14
#define SDATAOUT_RESET_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
