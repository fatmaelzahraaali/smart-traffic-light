/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define ArdOn_Pin GPIO_PIN_1
#define ArdOn_GPIO_Port GPIOA
#define ArdOff_Pin GPIO_PIN_2
#define ArdOff_GPIO_Port GPIOA
#define UsOn_Pin GPIO_PIN_0
#define UsOn_GPIO_Port GPIOB
#define UsOff_Pin GPIO_PIN_1
#define UsOff_GPIO_Port GPIOB
#define NodeIN_Pin GPIO_PIN_11
#define NodeIN_GPIO_Port GPIOB
#define NodeIN_EXTI_IRQn EXTI15_10_IRQn
#define TestLed_Pin GPIO_PIN_12
#define TestLed_GPIO_Port GPIOB
#define CAM_4_Pin GPIO_PIN_4
#define CAM_4_GPIO_Port GPIOB
#define CAM_3_Pin GPIO_PIN_5
#define CAM_3_GPIO_Port GPIOB
#define CAM_2_Pin GPIO_PIN_6
#define CAM_2_GPIO_Port GPIOB
#define CAM_1_Pin GPIO_PIN_7
#define CAM_1_GPIO_Port GPIOB
#define CEpin_Pin GPIO_PIN_8
#define CEpin_GPIO_Port GPIOB
#define CSNpin_Pin GPIO_PIN_9
#define CSNpin_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
