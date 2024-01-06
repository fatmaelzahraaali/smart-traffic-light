/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_nrf24.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CSNpin_Pin GPIO_PIN_9
#define CEpin_Pin GPIO_PIN_8

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t TxpipeAddrs = 0x1122334455;
char rec[32] = "";

uint8_t Ard_flag = 1;
uint8_t US_flag = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	NRF24_begin(GPIOB, CSNpin_Pin, CEpin_Pin, hspi1);
	NRF24_openReadingPipe(0, TxpipeAddrs);
	NRF24_setAutoAck(false);
	NRF24_setPayloadSize(32);
	NRF24_setChannel(76);
	NRF24_startListening();

	HAL_GPIO_WritePin(GPIOB, TestLed_Pin, 1);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB, TestLed_Pin, 0);

	HAL_GPIO_WritePin(GPIOB, CAM_1_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, CAM_2_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, CAM_3_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, CAM_4_Pin, 0);

	HAL_GPIO_WritePin(GPIOB, UsOn_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, UsOff_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, ArdOn_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, ArdOff_Pin, 0);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		NRF24_read(rec, 32);
		if (rec[0] == '8') {

			for (uint8_t i = 0; i < 5; i++)
			{
				HAL_GPIO_WritePin(GPIOB, TestLed_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, TestLed_Pin, 0);
				HAL_Delay(300);
			}
			if (rec[1] == '1') {
				HAL_GPIO_WritePin(GPIOB, CAM_1_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, CAM_1_Pin, 0);
			} else if (rec[1] == '2') {
				HAL_GPIO_WritePin(GPIOB, CAM_2_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, CAM_2_Pin, 0);
			} else if (rec[1] == '3') {
				HAL_GPIO_WritePin(GPIOB, CAM_3_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, CAM_3_Pin, 0);
			} else if (rec[1] == '4') {
				HAL_GPIO_WritePin(GPIOB, CAM_4_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, CAM_4_Pin, 0);
			}
			if(Ard_flag == 1)
			{
				/****Pulse to turn off Arduino****/
				HAL_GPIO_WritePin(GPIOA, ArdOff_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOA, ArdOff_Pin, 0);
				Ard_flag = 0;

				HAL_Delay(2000);

				/****Pulse to turn on us system****/
				HAL_GPIO_WritePin(GPIOB, UsOn_Pin, 1);
				HAL_Delay(300);
				HAL_GPIO_WritePin(GPIOB, UsOn_Pin,0);
				US_flag = 1;
			}

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, ArdOn_Pin | ArdOff_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			UsOn_Pin | UsOff_Pin | TestLed_Pin | CAM_4_Pin | CAM_3_Pin
					| CAM_2_Pin | CAM_1_Pin | CEpin_Pin | CSNpin_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : ArdOn_Pin ArdOff_Pin */
	GPIO_InitStruct.Pin = ArdOn_Pin | ArdOff_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : UsOn_Pin UsOff_Pin TestLed_Pin CAM_4_Pin
	 CAM_3_Pin CAM_2_Pin CAM_1_Pin CEpin_Pin
	 CSNpin_Pin */
	GPIO_InitStruct.Pin = UsOn_Pin | UsOff_Pin | TestLed_Pin | CAM_4_Pin
			| CAM_3_Pin | CAM_2_Pin | CAM_1_Pin | CEpin_Pin | CSNpin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : NodeIN_Pin */
	GPIO_InitStruct.Pin = NodeIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NodeIN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (Ard_flag == 0) {
		/****Pulse to turn off us system****/
		HAL_GPIO_WritePin(GPIOB, UsOff_Pin, 1);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOB, UsOff_Pin, 0);
		US_flag = 0;
		HAL_Delay(2000);
		/****Pulse to turn off Arduino****/
		HAL_GPIO_WritePin(GPIOA, ArdOn_Pin, 1);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOA, ArdOn_Pin, 0);
		Ard_flag = 1;

	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/*
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
