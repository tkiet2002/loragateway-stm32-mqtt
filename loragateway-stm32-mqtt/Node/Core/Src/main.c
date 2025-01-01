/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "sx1278.h"
#include "stdio.h"
#include "math.h"
#include <stdlib.h>
#define  id_node 1
#define  rx 0
#define  tx 1
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define OUTPUT_PIN_B11 (*(volatile uint32_t *)0x422181AC) // Bit 11 của GPIOB_ODR
#define OUTPUT_PIN_B12 (*(volatile uint32_t *)0x422181B0) // Bit 12 của GPIOB_ODR
#define OUTPUT_PIN_B13 (*(volatile uint32_t *)0x422181B4)

#define RELAY 		OUTPUT_PIN_B11
#define LED 		OUTPUT_PIN_B12
#define TEST 		OUTPUT_PIN_B13
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxDt[3];
volatile uint8_t relay_status = 0, mode_hien_tai = 0;
char tx_lora_bf[21] = { };
void lora_node_send() {
	if (RELAY == 0)
		tx_lora_bf[16] = 1;
	else
		tx_lora_bf[16] = 0;
	standby_mode();
	writeRegister(0x0D, 0);
	writeRegister(0x22, 0);
	writeRegister(0, id_node - 1);
	for (unsigned char i = 0; i < 18; i++) {
		writeRegister(0, tx_lora_bf[i]);
	}
	writeRegister(0x22, 18);
	tx_mode();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_2:
		if (mode_hien_tai == rx) {
			writeRegister(0x12, readRegister(0x12) | 0x40); // xoa co
			writeRegister(0x0d, readRegister(0x10)); // dua con tro ve vi tri dau
			RxDt[0] = readRegister(0);
			RxDt[1] = readRegister(0);
			RxDt[2] = readRegister(0);
			if(RxDt[0] == (id_node - 1))
			{
				if (RxDt[1] == 0xFD)
				{
					LED = 0;
					writeRegister(0x40, 0X40); // anh xa tx done
					lora_node_send();
					mode_hien_tai = tx;
					////////////////them thoi gian timeout vao day
				}
				else if(RxDt[1] == 0xFE)
				{
					LED = 0;
					relay_status = RxDt[2];
					if ((relay_status & 0x80) == 0x80) {
						RELAY = ~(relay_status & 0x01); /// vi relay kich muc 0
					}
					mode_hien_tai = tx;
				}
				else;
			}
		} else {
			LED = 1;
			writeRegister(0x12, 0x08);
			writeRegister(0x40, 0X00); // anh xa rx done
			rx_mode();
			mode_hien_tai = rx;
		}
		break;
	case GPIO_PIN_4:
		RELAY = 0;
		break;
	case GPIO_PIN_13:
		RELAY = 1;
		break;
	}
}

// bien lien quan ADC
uint16_t somau = 0, somau_max = 300;
int32_t mang_adc1[300] = { }, adc_tong1 = 0;
int32_t mang_adc2[300] = { }, adc_tong2 = 0;
int32_t mang_adc3[300] = { }, adc_tong3 = 0;
int32_t mang_adc4[300] = { }, adc_tong4 = 0;

float mau_dong_dien1[150] = { };
float mau_dong_dien2[150] = { };
float mau_dong_dien3[150] = { };
float mau_dong_dien4[150] = { };

float tong_dong1 = 0, irms1 = 0;
float tong_dong2 = 0, irms2 = 0;
float tong_dong3 = 0, irms3 = 0;
float tong_dong4 = 0, irms4 = 0;

float irms[4] = { };

uint32_t adc_values[4];

//////////////////////////////

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	RELAY = 1;
	HAL_I2C_Init(&hi2c1);
	HAL_SPI_Init(&hspi1);
	setbuf(stdout, NULL);
	oled_init();
	oled_display_pattern();
	sx1278_init(0x6c4000);
	rx_mode();
	HAL_TIM_Base_Start(&htim1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, adc_values, 4);
	oled_set_cursor(0, 0);
	printf("NODE_%d", (uint8_t) id_node);
	LED = 1;

//	HAL_Delay(1000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		somau = 0;
		while (somau < somau_max) {
			__HAL_TIM_SET_COUNTER(&htim1, 0);
			mang_adc1[somau] = adc_values[0];
			mang_adc2[somau] = adc_values[1];
			mang_adc3[somau] = adc_values[2];
			mang_adc4[somau] = adc_values[3];
			somau++;
			while (__HAL_TIM_GET_COUNTER(&htim1) < 200) {
			}
		}
/////////////////////////////////////////
//		for (unsigned int i = 0; i < somau_max; i = i + 2) {
//			if (mang_adc1[i] > 2048 && mang_adc1[i + 1] > 2048) {
//				if (mang_adc1[i] < mang_adc1[i + 1])
//					mang_adc1[i / 2] = mang_adc1[i];
//				else
//					mang_adc1[i / 2] = mang_adc1[i + 1];
//
//			} else if (mang_adc1[i] == 2048 && mang_adc1[i + 1] == 2048)
//				mang_adc1[i / 2] = 2048;
//
//			else if (mang_adc1[i] < 2048 && mang_adc1[i + 1] < 2048) {
//				if (mang_adc1[i] < mang_adc1[i + 1])
//					mang_adc1[i / 2] = mang_adc1[i + 1];
//				else
//					mang_adc1[i / 2] = mang_adc1[i];
//			} else {
//				if ((abs(2048 - mang_adc1[i])) > (abs(2048 - mang_adc1[i + 1])))
//					mang_adc1[i / 2] = mang_adc1[i + 1];
//				else
//					mang_adc1[i / 2] = mang_adc1[i];
//
//			}
//		}
//
//		for (unsigned int i = 0; i < somau_max / 2; i = i + 2) {
//			mang_adc1[i / 2] = (mang_adc1[i] + mang_adc1[i + 1]) / 2;
//			mau_dong_dien1[i / 2] = (((float) mang_adc1[i / 2] * (3.3 / 4096.0))
//					- 1.65) / 0.02;
//		}

		for (unsigned int i = 0; i < somau_max; i = i + 2) {
			mang_adc1[i / 2] = (mang_adc1[i] + mang_adc1[i + 1]) / 2;
			mau_dong_dien1[i / 2] = (((float) mang_adc1[i / 2] * (3.3 / 4096.0))
					- 1.65) / 0.02;

			mang_adc2[i / 2] = (mang_adc2[i] + mang_adc2[i + 1]) / 2;
			mau_dong_dien2[i / 2] = (((float) mang_adc2[i / 2] * (3.3 / 4096.0))
					- 1.65) / 0.02;

			mang_adc3[i / 2] = (mang_adc3[i] + mang_adc3[i + 1]) / 2;
			mau_dong_dien3[i / 2] = (((float) mang_adc3[i / 2] * (3.3 / 4096.0))
					- 1.65) / 0.02;

			mang_adc4[i / 2] = (mang_adc4[i] + mang_adc4[i + 1]) / 2;
			mau_dong_dien4[i / 2] = (((float) mang_adc4[i / 2] * (3.3 / 4096.0))
					- 1.65) / 0.02;
		}

		tong_dong1 = 0;
		tong_dong2 = 0;
		tong_dong3 = 0;
		tong_dong4 = 0;

		for (unsigned int i = 0; i < somau_max / 2; i++) {
			tong_dong1 = tong_dong1 + mau_dong_dien1[i] * mau_dong_dien1[i];
			tong_dong2 = tong_dong2 + mau_dong_dien2[i] * mau_dong_dien2[i];
			tong_dong3 = tong_dong3 + mau_dong_dien3[i] * mau_dong_dien3[i];
			tong_dong4 = tong_dong4 + mau_dong_dien4[i] * mau_dong_dien4[i];
		}

		irms[0] = sqrt(tong_dong1 / 75.0) - 0.15;
		if (irms[0] < 0.2)
			irms[0] = 0;
		if (irms[0] > 50)
			irms[0] = 50;

		irms[1] = sqrt(tong_dong2 / 75.0) - 0.15;
		if (irms[1] < 0.2)
			irms[1] = 0;
		if (irms[1] > 50)
			irms[1] = 50;

		irms[2] = sqrt(tong_dong3 / 75.0) - 0.15;
		if (irms[2] < 0.2)
			irms[2] = 0;
		if (irms[2] > 50)
			irms[2] = 50;

		irms[3] = sqrt(tong_dong4 / 75.0) - 0.15;
		if (irms[3] < 0.2)
			irms[3] = 0;
		if (irms[3] > 50)
			irms[3] = 50;

//////////////////////////////////////

		oled_set_cursor(0, 0);
		printf("I1 : %05.2f A", irms[0]);
		oled_set_cursor(0, 2);
		printf("I2 : %05.2f A", irms[1]);
		oled_set_cursor(0, 4);
		printf("I3 : %05.2f A", irms[2]);
		oled_set_cursor(0, 6);
		printf("I4 : %05.2f A", irms[3]);

		sprintf(tx_lora_bf, "%04d%04d%04d%04d", (uint16_t) (irms[0] * 100),
				(uint16_t) (irms[1] * 100), (uint16_t) (irms[2] * 100),
				(uint16_t) (irms[3] * 100));

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB11 PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	ssd1306_char_f8x16(ch);
//	oled_display_char(ch);

	return ch;
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
