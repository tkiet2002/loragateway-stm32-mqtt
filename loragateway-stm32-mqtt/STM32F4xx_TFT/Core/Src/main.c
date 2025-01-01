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
#include "ili9488.h"
#include "xpt2046.h"
#include "stdio.h"
#include "stdbool.h"
#include "022_Open_Sans_Bold.h"
#include "024_Open_Sans_Bold.h"
#include "026_Open_Sans_Bold.h"
#include "sx1278.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUZZER_ON 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
#define BUZZER_OFF 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

#define _Open_Sans_Bold_22      &Open_Sans_Bold_22
#define _Open_Sans_Bold_24      &Open_Sans_Bold_24
#define _Open_Sans_Bold_26      &Open_Sans_Bold_26

#define RL_OFF 0u
#define RL_ON 1u
#define DISPLAY_SCREEN 0u
#define CTRL_SCREEN_1 1u
#define CTRL_SCREEN_2 2u
#define ER_LIMIT_VLUE 2u // neu so lan request lon hon thi coi nhu node khong hoat dong
#define INDEX_VALUE 10u
#define SIZEOF_DATABUFFER 40u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SET_BIT_USER(VAR, BIT) 		((VAR) |= (1U << (BIT)))
#define CLEAR_BIT_USER(VAR, BIT) 	((VAR) &= ~(1U << (BIT)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Variables used for TFT displays
volatile uint16_t touchx = 0, touchy = 0;
uint8_t Start_x = 20, Start_y = 20, Wigth_Bt = 130, High_Bt = 60;
char Buffer[20];
char buf1[20];
uint16_t Posx_Node[3] = { 20, 176, 320 };
uint8_t Posy_Node[3] = { 20, 95, 170 };
uint8_t PoStart_x = 20, PoStart_y = 37;

uint8_t StDisplay = 0; // DISPLAY_SCREEN = 0u - CTRL_SCREEN_1 = 1u - CTRL_SCREEN_2 = 2u
bool Display_Updated_Flag = false;

uint8_t RelayStatus_arr_ESP[INDEX_VALUE];
uint8_t ReL_LoRa_Sen[INDEX_VALUE];
uint16_t Ctr_Dev = 0;
uint16_t Ctr_Dev_ESP = 0;
uint16_t Ctr_Dev_Old = 0;
uint16_t StNode = 0;
uint16_t StNode_Old = 0;
uint8_t Buzzer = 0;
uint8_t Buzzer_Old = 0;

volatile uint8_t SPI2_TX_completed_flag = 1;
volatile uint16_t debounce_100ms;
uint8_t timer_counter = 0;
volatile bool timer_cnt3 = false;

bool Flag = false; // Flag used to allow data to be sent the first time
volatile bool Flag_Uart = false;

uint8_t tx_lora_bf[3] = { }; // bo dem gui
uint8_t data_node[INDEX_VALUE][20] = { }; // luu du lieu chua xu ly
uint8_t Node_IsCommunicating = 0;
uint8_t request_error[INDEX_VALUE];
uint8_t check_node[INDEX_VALUE];

uint8_t RelayStatus_arr_STM[INDEX_VALUE]; ///trang thai relay tai cac node
uint8_t NodeStatus_arr[INDEX_VALUE]; ///trang thai cac node
float irms1_arr[INDEX_VALUE]; // dong cua cam bien 1
float irms1_old[INDEX_VALUE];

uint8_t Tx_Buf[SIZEOF_DATABUFFER];

uint8_t start_byte_data = 0x83;
uint8_t start_byte_ctrl[2] = {0x83, 0x42};
uint8_t start_byte_warning[2] = {0x83, 0x57};
uint8_t start_byte_status[2] = {0x83, 0x54};
uint8_t stop_byte = 0x50;
uint8_t byte_rx;
uint8_t ena_rx = 0;
uint8_t receivedData[3], Rx_Idx = 0;
bool index_data_changed[INDEX_VALUE];
bool stDataChanged = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Send_Lora_UpdateRelay(uint8_t node);
void Update_StatusBt(uint8_t *New_St_Relay, uint8_t *Old_St_Relay);
void Check_Uart_update(uint8_t *RelayStatus_arr_ESP_Para, uint8_t *RelayStatus_arr_STM_Para);
bool isIrmsChanged(float *currentData, float *lastData, bool *idx_updated);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/***************************************************************************************
 ************* The function is used to check for the button status change **************
*****************************************************************************************/
void Check_Uart_update(uint8_t *RelayStatus_arr_ESP_Para, uint8_t *RelayStatus_arr_STM_Para)
{
	uint8_t idx_relay;
	while(Flag_Uart == true)
	{
		for(idx_relay = 0; idx_relay < INDEX_VALUE; idx_relay++)
		{
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			RelayStatus_arr_ESP_Para[idx_relay] = (Ctr_Dev & (1u << idx_relay)) ? RL_ON : RL_OFF;
			if(RelayStatus_arr_ESP_Para[idx_relay] != RelayStatus_arr_STM_Para[idx_relay])
			{
				ReL_LoRa_Sen[idx_relay] = RelayStatus_arr_ESP_Para[idx_relay] | 0x80;
				Send_Lora_UpdateRelay(idx_relay);
				if(StDisplay == CTRL_SCREEN_1)
				{
					Update_StatusBt(&RelayStatus_arr_ESP_Para[0], &RelayStatus_arr_STM_Para[0]);
				}
			}
			RelayStatus_arr_STM_Para[idx_relay] = RelayStatus_arr_ESP_Para[idx_relay];
			if(idx_relay >= 9)
			{
				Flag_Uart = false;
				HAL_UART_Receive_IT(&huart1, &byte_rx, 1);
			}
		}
	}
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	SPI2_TX_completed_flag = 1;
}

void Send_Uart1(uint8_t *start_byte, uint8_t *Buffer, uint8_t *stop_bytes, uint8_t SizeofStartByte, uint8_t SizeofBuff)
{
    HAL_UART_Transmit(&huart1, start_byte, SizeofStartByte, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, Buffer, SizeofBuff, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, stop_bytes, 1, HAL_MAX_DELAY);
}

/***************************************************************************************
 ************* The function is used to check for the current value change **************
****************************************************************************************/
bool isIrmsChanged(float *currentData, float *lastData, bool *idx_updated)
{
    bool isChanged = false;
    uint8_t idx_update_vl;

    for (idx_update_vl = 0; idx_update_vl < INDEX_VALUE; idx_update_vl++)
    {
        if ((lastData[idx_update_vl] - currentData[idx_update_vl] >= 0.25)
            || (currentData[idx_update_vl] - lastData[idx_update_vl] >= 0.25))
        {
            currentData[idx_update_vl] = lastData[idx_update_vl];
            idx_updated[idx_update_vl] = true;
            isChanged = true;
        }
        else
        {
            idx_updated[idx_update_vl] = false;
        }
    }
    return isChanged;
}
/***************************************************************************************
 	 ************* The function is used to display the current value **************
*****************************************************************************************/
void Default_Display() {
	char buf[20];
	int col_width = 220, row_height = 50, start_x = 15, start_y = 5, line_thickness = 3;
	int i;
	ILI9341_Fill_Screen(WHITE);

	/* ---- Create table ---- */
	for (i = 0; i <= 5; i++) {
		fillRect(start_x, start_y + i * row_height, col_width * 2,
				line_thickness, GREY15);
	}
	for (i = 0; i <= 2; i++) {
		fillRect(start_x + i * col_width, start_y, line_thickness,
				row_height * 5 + line_thickness, GREY15);
	}

	/* ---- Print Current value ---- */
	for (i = 0; i < 5; i++) {
		if(NodeStatus_arr[i] == 1)
		{
			fillRect(start_x + line_thickness, start_y + i * row_height + line_thickness,
					col_width - line_thickness * 2, row_height - line_thickness * 2, WHITE);
			sprintf(buf, "Node %d: %0.2fA", i + 1, irms1_arr[i]);
			LCD_Font(start_x + 12, start_y + i * row_height + 35, buf, _Open_Sans_Bold_26, 1, ILI9488_MAGENTA);
		}
		else
		{
			fillRect(start_x + line_thickness, start_y + i * row_height + line_thickness,
					col_width - line_thickness * 2, row_height - line_thickness * 2, WHITE);
			sprintf(buf, "Node %d: TAT", i + 1);
			LCD_Font(start_x + 12, start_y + i * row_height + 35, buf, _Open_Sans_Bold_26, 1, ILI9488_MAGENTA);
		}

		if(NodeStatus_arr[i + 5] == 1)
		{
			fillRect(start_x + col_width + line_thickness, start_y + i * row_height + line_thickness,
					col_width - line_thickness * 2, row_height - line_thickness * 2, WHITE);
			sprintf(buf, "Node %d: %.2fA", i + 6, irms1_arr[i + 5]);
			LCD_Font(start_x + 12 + col_width, start_y + i * row_height + 35, buf, _Open_Sans_Bold_26, 1, ILI9488_MAROON);

		}
		else
		{
			fillRect(start_x + col_width + line_thickness, start_y + i * row_height + line_thickness,
					col_width - line_thickness * 2, row_height - line_thickness * 2, WHITE);
			sprintf(buf, "Node %d: TAT", i + 6);
			LCD_Font(start_x + 12 + col_width, start_y + i * row_height + 35, buf, _Open_Sans_Bold_26, 1, ILI9488_MAROON);
		}
	}
	/*---- Create Setting Button ----*/
	LCD_Rect_Round_Fill(100, 265, 270, 50, 5, GREEN);
	LCD_Font(170, 298, "DIEU KHIEN", _Open_Sans_Bold_22, 1, BLACK);
}
/***************************************************************************************
 	********** The function is used to check and update the button status **********
*****************************************************************************************/
void Update_StatusBt(uint8_t *New_St_Relay, uint8_t *Old_St_Relay)
{
	uint8_t idx_st_relay;
	for(idx_st_relay = 0; idx_st_relay < 10; idx_st_relay++)
	{
		if(New_St_Relay[idx_st_relay] != Old_St_Relay[idx_st_relay])
		{
			Old_St_Relay[idx_st_relay] = New_St_Relay[idx_st_relay];
			if(idx_st_relay < 3)
			{
				if (0 == Old_St_Relay[idx_st_relay])
				{
					LCD_Rect_Round_Fill(Start_x + (idx_st_relay * Wigth_Bt * 1.2), Start_y, Wigth_Bt, High_Bt, 5, ILI9488_RED);
					sprintf(Buffer, "N%d: TAT", idx_st_relay + 1);
				}
				else
				{
					LCD_Rect_Round_Fill(Start_x + (idx_st_relay * Wigth_Bt * 1.2), Start_y, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
					sprintf(Buffer, "N%d: BAT", idx_st_relay + 1);
				}
				LCD_Font(Start_x + (idx_st_relay * Wigth_Bt * 1.2) + PoStart_x, Start_y + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
			}
			else if((idx_st_relay >= 3) && (idx_st_relay < 6))
			{
				if (0 == Old_St_Relay[idx_st_relay])
				{
					LCD_Rect_Round_Fill(Start_x + ((idx_st_relay - 3) * Wigth_Bt * 1.2), Start_y + 75, Wigth_Bt, High_Bt, 5, ILI9488_RED);
					sprintf(Buffer, "N%d: TAT", idx_st_relay + 1);
				}
				else
				{
					LCD_Rect_Round_Fill(Start_x + ((idx_st_relay - 3) * Wigth_Bt * 1.2), Start_y + 75, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
					sprintf(Buffer, "N%d: BAT", idx_st_relay + 1);
				}
				LCD_Font(Start_x + ((idx_st_relay - 3) * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
			}
			else if((idx_st_relay >= 6) && (idx_st_relay < 9))
			{
				if (0 == Old_St_Relay[idx_st_relay])
				{
					LCD_Rect_Round_Fill(Start_x + ((idx_st_relay - 6) * Wigth_Bt * 1.2), Start_y + 75 * 2, Wigth_Bt, High_Bt, 5, ILI9488_RED);
					sprintf(Buffer, "N%d: TAT", idx_st_relay + 1);
				}
				else
				{
					LCD_Rect_Round_Fill(Start_x + ((idx_st_relay - 6) * Wigth_Bt * 1.2), Start_y + 75 * 2, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
					sprintf(Buffer, "N%d: BAT", idx_st_relay + 1);
				}
				LCD_Font(Start_x + ((idx_st_relay - 6) * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 * 2 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
			}
			else
			{
				if (0 == Old_St_Relay[idx_st_relay])
				{
					LCD_Rect_Round_Fill(Start_x + ((idx_st_relay - 9) * Wigth_Bt * 1.2), Start_y + 75 * 3, Wigth_Bt, High_Bt, 5, ILI9488_RED);
					sprintf(Buffer, "N%d: TAT", idx_st_relay + 1);
				}
				else
				{
					LCD_Rect_Round_Fill(Start_x + ((idx_st_relay - 9) * Wigth_Bt * 1.2), Start_y + 75 * 3, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
					sprintf(Buffer, "N%d: BAT", idx_st_relay + 1);
				}
				LCD_Font(Start_x + ((idx_st_relay - 9) * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 * 3 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
			}
		}
	}
}
/***************************************************************************************
 	 ************* The function is used to display the window control 2 **************
*****************************************************************************************/
void Screen2_Ctrl(void)
{
	setAddrWindow(0, 0, 320, 480);
	ILI9341_Fill_Screen(WHITE);
	/* -------------------- Button 'BAT HET' -------------*/
	LCD_Rect_Round_Fill(Start_x + 20, Start_y, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_OLIVE);
	LCD_Font(75, 72, "BAT HET", _Open_Sans_Bold_26, 1, BLACK);

	/* -------------------- Button 'BAT HET' -------------*/
	LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_OLIVE);
	LCD_Font(Start_x + Wigth_Bt + 145, 72, "TAT HET", _Open_Sans_Bold_26, 1, BLACK);

	/* -------------------- Button 'QUAY LAI' -------------*/
	LCD_Rect_Round_Fill(Start_x + 20, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, GREEN);
	LCD_Font(90, Start_y * 2 + 1.2 * High_Bt * 2, "<<===", _Open_Sans_Bold_26, 1, BLACK);

	if (0 == Buzzer)
	{
		LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_RED);
		LCD_Font(Start_x + Wigth_Bt + 140, Start_y * 2 + 1.2 * High_Bt * 2, "COI: TAT", _Open_Sans_Bold_26, 1, BLACK);
	}
	else
	{
		LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_PINK);
		LCD_Font(Start_x + Wigth_Bt + 140, Start_y * 2 + 1.2 * High_Bt * 2, "COI: BAT", _Open_Sans_Bold_26, 1, BLACK);
	}
	/*---- Create Setting Button ----*/
	LCD_Rect_Round_Fill(100, 245, 270, 60, 5, GREEN);
	LCD_Font(185, 285, "TRO VE", _Open_Sans_Bold_26, 1, BLACK);
}
/***************************************************************************************
 	 ************* The function is used to display the window control 1 **************
*****************************************************************************************/
void Screen1_Ctrl(void) {
	setAddrWindow(0, 0, 320, 480);
	ILI9341_Fill_Screen(WHITE);
	uint8_t i;
	for (i = 0; i < 3; i++)
	{
		/* ---- Check-update status ReL1-3 ---- */
		if (0 == RelayStatus_arr_STM[i])
		{
			LCD_Rect_Round_Fill(Start_x + (i * Wigth_Bt * 1.2), Start_y, Wigth_Bt, High_Bt, 5, ILI9488_RED);
			sprintf(Buffer, "N%d: TAT", i + 1);
			LCD_Font(Start_x + (i * Wigth_Bt * 1.2) + PoStart_x, Start_y + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
		}
		else
		{
			LCD_Rect_Round_Fill(Start_x + (i * Wigth_Bt * 1.2), Start_y, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
			sprintf(Buffer, "N%d: BAT", i + 1);
			LCD_Font(Start_x + (i * Wigth_Bt * 1.2) + PoStart_x, Start_y + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
		}
		/* ---- Check-update status ReL4-6 ---- */
		if (0 == RelayStatus_arr_STM[i + 3]) {
			LCD_Rect_Round_Fill(Start_x + (i * Wigth_Bt * 1.2), Start_y + 75, Wigth_Bt, High_Bt, 5, ILI9488_RED);
			sprintf(Buffer, "N%d: TAT", i + 3 + 1);
			LCD_Font(Start_x + (i * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
		}
		else
		{
			LCD_Rect_Round_Fill(Start_x + (i * Wigth_Bt * 1.2), Start_y + 75, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
			sprintf(Buffer, "N%d: BAT", i + 3 + 1);
			LCD_Font(Start_x + (i * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
		}
		/* ---- Check-update status ReL7-9 ---- */
		if (0 == RelayStatus_arr_STM[i + 6])
		{
			LCD_Rect_Round_Fill(Start_x + (i * Wigth_Bt * 1.2), Start_y + 75 * 2, Wigth_Bt, High_Bt, 5, ILI9488_RED);
			sprintf(Buffer, "N%d: TAT", i + 6 + 1);
			LCD_Font(Start_x + (i * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 * 2 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
		}
		else
		{
			LCD_Rect_Round_Fill(Start_x + (i * Wigth_Bt * 1.2), Start_y + 75 * 2, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
			sprintf(Buffer, "N%d: BAT", i + 6 + 1);
			LCD_Font(Start_x + (i * Wigth_Bt * 1.2) + PoStart_x, Start_y + 75 * 2 + PoStart_y, Buffer, _Open_Sans_Bold_22, 1, BLACK);
		}
	}
	/* ---- Check-update status ReL10 ---- */
	if (0 == RelayStatus_arr_STM[9])
	{
		LCD_Rect_Round_Fill(Start_x, Start_y + 75 * 3, Wigth_Bt, High_Bt, 5, ILI9488_RED);
		LCD_Font(Start_x + PoStart_x, Start_y + PoStart_y + 75 * 3, "N10: TAT", _Open_Sans_Bold_22, 1, BLACK);
	}
	else
	{
		LCD_Rect_Round_Fill(Start_x, Start_y + 75 * 3, Wigth_Bt, High_Bt, 5, ILI9488_PINK);
		LCD_Font(Start_x + PoStart_x, Start_y + PoStart_y + 75 * 3, "N10: BAT", _Open_Sans_Bold_22, 1, BLACK);
	}
	/* ---- Next Button ----*/
	LCD_Rect_Round_Fill(Start_x + (Wigth_Bt * 1.2), Start_y + 75 * 3, Wigth_Bt, High_Bt, 5, GREEN);
	LCD_Font(Start_x + (1 * Wigth_Bt * 1.2) + PoStart_x, Start_y + PoStart_y + 75 * 3, "TRO VE", _Open_Sans_Bold_22, 1, BLACK);

	/* ---- Back Button ---- */
	LCD_Rect_Round_Fill(Start_x + (Wigth_Bt * 1.2 * 2), Start_y + 75 * 3, Wigth_Bt, High_Bt, 5, GREEN);
	LCD_Font(Start_x + (2 * Wigth_Bt * 1.2) + PoStart_x + 5, Start_y + 75 * 3 + PoStart_y, "===>>", _Open_Sans_Bold_24, 1, BLACK);

}
/***************************************************************************************
 	 	 ************* The function is used to control the devices **************
*****************************************************************************************/
void Touch_XY() {
	uint8_t Ctr_Dev_Temp[2] = {0,0};
	/* ---- Press button 'DIEU KHIEN' ---- */
	if (touchx >= 100 && touchx <= 340 && touchy >= 230 && touchy <= 315)
	{
		if(StDisplay == DISPLAY_SCREEN)
		{
			StDisplay = CTRL_SCREEN_1;
			Screen1_Ctrl();
		}
	}
	/* ---- Press button 'TRO VE' ---- */
	if(touchx >= 200 && touchx <= 280 && touchy >= 255 && touchy <= 315)
	{
		if(StDisplay == CTRL_SCREEN_1)
		{
			StDisplay = DISPLAY_SCREEN;
			Default_Display();
		}
	}
	/* ---- Press button '===>>' ---- */
	if(touchx >= 335 && touchx <= 440 && touchy >= 255 && touchy <= 315)
	{
		if(StDisplay == CTRL_SCREEN_1)
		{
			StDisplay = CTRL_SCREEN_2;
			Screen2_Ctrl();
		}
	}
	if(StDisplay == CTRL_SCREEN_2)
	{
		uint8_t Ctr_Dev_Temp[2] = {0};
		/* ---- Press button 'BAT HET' ---- */
		if(touchx >= 60 && touchx <= 210 && touchy >= 45 && touchy <= 120)
		{
			Ctr_Dev = 0x03ff;
			Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
			Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
			Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2); /* Send Ctrl_Dev variable */
			if(StDisplay == CTRL_SCREEN_1)
			{
				Update_StatusBt(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
			}
			Ctr_Dev_Old = Ctr_Dev;
			Flag_Uart = true;
		}
		/* ---- Press button 'TAT HET' ---- */
		if(touchx >= 270 && touchx <= 425 && touchy >= 45 && touchy <= 120)
		{
			Ctr_Dev = 0x0000;
			Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
			Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
			Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2); /* Send Ctrl_Dev variable */
			if(StDisplay == CTRL_SCREEN_1)
			{
				Update_StatusBt(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
			}
			Ctr_Dev_Old = Ctr_Dev;
			Flag_Uart = true;
		}
		/* ---- Press button '<<===' ---- */
		if(touchx >= 60 && touchx <= 210 && touchy >= 190 && touchy <= 230)
		{
			StDisplay = CTRL_SCREEN_1;
			Screen1_Ctrl();
		}
		/* ---- Press button 'COI' ---- */
		if(touchx >= 270 && touchx <= 425 && touchy >= 190 && touchy <= 230)
		{
			debounce_100ms = 0;
			while (debounce_100ms < 30);
			Buzzer = !Buzzer;
			if (0 == Buzzer)
			{
				BUZZER_OFF;
				LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_RED);
				LCD_Font(Start_x + Wigth_Bt + 140, Start_y * 2 + 1.2 * High_Bt * 2, "COI: TAT", _Open_Sans_Bold_26, 1, BLACK);
			}
			else
			{
				BUZZER_ON;
				LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_PINK);
				LCD_Font(Start_x + Wigth_Bt + 140, Start_y * 2 + 1.2 * High_Bt * 2, "COI: BAT", _Open_Sans_Bold_26, 1, BLACK);
			}
			Send_Uart1(&start_byte_warning[0], &Buzzer, &stop_byte, 2, 1);
			Buzzer_Old = Buzzer;
		}
		/* ---- Press button 'TRO VE' ---- */
		if(touchx >= 90 && touchx <= 345 && touchy >= 245 && touchy <= 315)
		{
			StDisplay = DISPLAY_SCREEN;
			Default_Display();
		}
	}

	else if(StDisplay == CTRL_SCREEN_1)
	{
		/* ---- Press Buttons 'NODE' ---- */
		for (int i = 0; i < 3; i++)
		{
			if (touchx >= (Posx_Node[i] + PoStart_x) && touchx <= (Wigth_Bt + Posx_Node[i] - PoStart_x)
					&& touchy >= (50) && touchy <= (80))
			{
				debounce_100ms = 0;
				while (debounce_100ms < 30);
				RelayStatus_arr_ESP[i] = !RelayStatus_arr_ESP[i];
				ReL_LoRa_Sen[i] = RelayStatus_arr_ESP[i] | 0x80;
				Send_Lora_UpdateRelay(i);
				if(RelayStatus_arr_ESP[i] == 0)
				{
					CLEAR_BIT_USER(Ctr_Dev, i);
				}
				else
				{
					SET_BIT_USER(Ctr_Dev, i);
				}
				Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
				Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
				Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2);
				if(StDisplay == CTRL_SCREEN_1)
				{
					Update_StatusBt(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
				}
				Ctr_Dev_Old = Ctr_Dev;
				break;
			}

			if (touchx >= (Posx_Node[i] + PoStart_x) && touchx <= (Wigth_Bt + Posx_Node[i] - PoStart_x)
					&& touchy >= (130) && touchy <= (160))
			{
				debounce_100ms = 0;
				while (debounce_100ms < 30);
				RelayStatus_arr_ESP[i + 3] = !RelayStatus_arr_ESP[i + 3];
				ReL_LoRa_Sen[i + 3] = RelayStatus_arr_ESP[i + 3] | 0x80;
				Send_Lora_UpdateRelay(i+3);
				if(RelayStatus_arr_ESP[i] == 0)
				{
					CLEAR_BIT_USER(Ctr_Dev, i + 3);
				}
				else
				{
					SET_BIT_USER(Ctr_Dev, i + 3);
				}
				Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
				Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
				Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2);
				if(StDisplay == CTRL_SCREEN_1)
				{
					Update_StatusBt(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
				}
				Ctr_Dev_Old = Ctr_Dev;
				break;
			}
			if (touchx >= (Posx_Node[i] + PoStart_x) && touchx <= (Wigth_Bt + Posx_Node[i] - PoStart_x)
					&& touchy >= (200) && touchy <= (230))
			{
				debounce_100ms = 0;
				while (debounce_100ms < 30); /* Chong doi nut nhan */
				RelayStatus_arr_ESP[i + 6] = !RelayStatus_arr_ESP[i + 6];
				ReL_LoRa_Sen[i + 6] = RelayStatus_arr_ESP[i + 6] | 0x80;
				Send_Lora_UpdateRelay(i + 6);
				if(RelayStatus_arr_ESP[i] == 0)
				{
					CLEAR_BIT_USER(Ctr_Dev, i + 6);
				}
				else
				{
					SET_BIT_USER(Ctr_Dev, i + 6);
				}
				Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
				Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
				Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2);
				if(StDisplay == CTRL_SCREEN_1)
				{
					Update_StatusBt(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
				}
				Ctr_Dev_Old = Ctr_Dev;
				break;
			}
			if (touchx >= (Posx_Node[0] + PoStart_x) && touchx <= (Wigth_Bt + Posx_Node[0] - PoStart_x)
					&& touchy >= (250) && touchy <= (280))
			{
				debounce_100ms = 0;
				while (debounce_100ms < 30); /* Chong doi nut nhan */
				RelayStatus_arr_ESP[9] = !RelayStatus_arr_ESP[9];
				ReL_LoRa_Sen[9] = RelayStatus_arr_ESP[9] | 0x80;
				Send_Lora_UpdateRelay(9);
				if(RelayStatus_arr_ESP[9] == 0)
				{
					CLEAR_BIT_USER(Ctr_Dev, 9);
				}
				else
				{
					SET_BIT_USER(Ctr_Dev, 9);
				}
				Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
				Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
				Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2);
				if(StDisplay == CTRL_SCREEN_1)
				{
					Update_StatusBt(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
				}
				Ctr_Dev_Old = Ctr_Dev;
				break;
			}
		}
	}
	else;
}
/***************************************************************************************
  ************* The function is used to update the current value change **************
*****************************************************************************************/
void Update_Current_Val(bool *idx_changed)
{
    char buf[20];
    int x_positions[] = {140, 140, 140, 140, 140, 360, 360, 360, 360};
    int y_positions[] = {15, 65, 115, 165, 215, 15, 65, 115, 165};
    int y_texts[] = {40, 90, 140, 190, 240, 40, 90, 140, 190};
    uint16_t colors[] = {ILI9488_MAGENTA, ILI9488_MAGENTA, ILI9488_MAGENTA, ILI9488_MAGENTA, ILI9488_MAGENTA,
                         ILI9488_MAROON, ILI9488_MAROON, ILI9488_MAROON, ILI9488_MAROON};
    int width = 90;
    int height = 30;

    for (int i = 0; i < 9; i++)
    {
        if (idx_changed[i] == 1)
        {
            fillRect(x_positions[i], y_positions[i], width, height, WHITE);
            sprintf(buf, "%0.2fA", irms1_arr[i]);
            LCD_Font(x_positions[i], y_texts[i], buf, _Open_Sans_Bold_26, 1, colors[i]);
        }
        else if (NodeStatus_arr[i] == 0)
        {
            fillRect(x_positions[i], y_positions[i], width, height, WHITE);
            LCD_Font(x_positions[i] + 1, y_texts[i], "TAT", _Open_Sans_Bold_26, 1, colors[i]);
        }
    }
    if(idx_changed[9] == 1)
	{
		fillRect(370, 215, 85, 30, WHITE);
		sprintf(buf, "%0.2fA", irms1_arr[9]);
		LCD_Font(370, 240, buf, _Open_Sans_Bold_26, 1, ILI9488_MAROON);
	}
	else if(NodeStatus_arr[9] == 0)
	{
		fillRect(370, 215, 85, 30, WHITE);
		LCD_Font(370, 240, "TAT", _Open_Sans_Bold_26, 1, ILI9488_MAROON);
	}
}
/***************************************************************************************
 	 ************* The function is used to send a request to node **************
*****************************************************************************************/
void Send_Request_Lora(uint8_t node)
{
    uint8_t i;
    standby_mode();
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0); // Reset kích thước payload
    tx_lora_bf[0] = node;
    tx_lora_bf[1] = 0xFD;
    tx_lora_bf[2] = 0xFF;
    for (i = 0; i < 3; i++)
    {
        writeRegister(REG_FIFO, tx_lora_bf[i]);
    }
    writeRegister(REG_PAYLOAD_LENGTH, 3);  // kích thước payload
    tx_mode();
}
/*************************************************************************************************
******* The function is used to send relay status to node when the relay status is updated *******
**************************************************************************************************/
void Send_Lora_UpdateRelay(uint8_t node)
{
	uint8_t i;
	uint32_t timeout = HAL_GetTick();
	uint32_t max_timeout = 50;
	sx1278_init(0x6c4000);
	standby_mode();
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	writeRegister(REG_PAYLOAD_LENGTH, 0);
	tx_lora_bf[0] = node;
	tx_lora_bf[1] = 0xFE;
	tx_lora_bf[2] = (ReL_LoRa_Sen[node]);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	for (i = 0; i < 3; i++)
	{
		writeRegister(0, tx_lora_bf[i]);
	}
	writeRegister(REG_PAYLOAD_LENGTH, 3);  // kích thước payload
	tx_mode();

	while (!(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)) // wait until for the data is sent completely
	{
		if((HAL_GetTick() - timeout) > max_timeout)
		{
			break;
		}
	}
	writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		touchx = getX();
		touchy = getY();
		debounce_100ms++;
	}
	else if (htim->Instance == TIM3)
	{
		timer_cnt3 = true;
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case GPIO_PIN_3:
			writeRegister(0x12, 0x08);
			rx_mode();
			break;
	}
}

/***********************************************************************
******* Function used to convert 4 byte data to float variable *******
************************************************************************/
void ConvertFloatAsBytes(float value, uint8_t *buffer)
{
    memcpy(buffer, &value, sizeof(value)); // Copy float bytes to buffer
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	if(byte_rx == 0x83)
    	{
    		ena_rx = 1;
			Rx_Idx = 0;
    	}
    	else if((byte_rx != 0x50) && (ena_rx == 1))
    	{
    		receivedData[Rx_Idx] = byte_rx;
			Rx_Idx++;
    	}
    	else if((byte_rx == 0x50) && (ena_rx == 1))
    	{
    		if(receivedData[0] == 0xFE)
    		{
    			Buzzer = receivedData[1];
    		}
    		else if(receivedData[0] == 0xFD)
    		{
    			Ctr_Dev = ((uint16_t)receivedData[1] << 8) | (uint16_t)receivedData[2];
    		}
    		ena_rx = 0;
    	}
    	if(Buzzer_Old != Buzzer)
		{
			if (0 == Buzzer)
			{
				BUZZER_OFF;
				LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_RED);
				LCD_Font(Start_x + Wigth_Bt + 140, Start_y * 2 + 1.2 * High_Bt * 2, "COI: TAT", _Open_Sans_Bold_26, 1, BLACK);
			}
			else
			{
				BUZZER_ON;
				LCD_Rect_Round_Fill(Start_x + Wigth_Bt + 110, Start_y + High_Bt + 50, Wigth_Bt + 50, High_Bt + 30, 5, ILI9488_PINK);
				LCD_Font(Start_x + Wigth_Bt + 140, Start_y * 2 + 1.2 * High_Bt * 2, "COI: BAT", _Open_Sans_Bold_26, 1, BLACK);
			}
			Buzzer_Old = Buzzer;
		}
    	/* Update new values */
		Ctr_Dev_Old = Ctr_Dev;
		Flag_Uart = true;
    }
}

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_TIM_Base_Start_IT(&htim2);
  ILI9488_Init();
  setRotation(1);
  sx1278_init(0x6c4000);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart1, &byte_rx, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Update Ctrl_Dev variable is received from ESP */
		Check_Uart_update(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);

		if((timer_cnt3 == true) && (Flag_Uart == false))
		{
			if ((readRegister(0x12) & 0x40) == 0x40)  //////////neu co nhan duoc du lieu
			{
				writeRegister(0x12, readRegister(0x12) | 0x40); // xoa co ngat
				writeRegister(0x0d, readRegister(0x10)); // dua ve vi tri dau trong bo dem de doc

				if (readRegister(0) == Node_IsCommunicating) // neu nhan dung gia tri cua node dang giao tiep
				{
					Check_Uart_update(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
					check_node[Node_IsCommunicating] = 1; // da nhan duoc du lieu thi set len 1
					NodeStatus_arr[Node_IsCommunicating] = 1; /// cho trang thai node dang giao tiep bang 1
					request_error[Node_IsCommunicating] = 0; /// reset so lan loi

					/* nhan du lieu vao bo dem*/
					for (unsigned char i = 0; i < 17; i++) {
						data_node[Node_IsCommunicating][i] = readRegister(0);
					}

					/*tinh gia tri dong cam bien 1*/
					uint16_t dong_nguyen = 0;
					dong_nguyen = (data_node[Node_IsCommunicating][0] - 48) * 1000
							+ (data_node[Node_IsCommunicating][1] - 48) * 100
							+ (data_node[Node_IsCommunicating][2] - 48) * 10
							+ (data_node[Node_IsCommunicating][3] - 48);
					irms1_arr[Node_IsCommunicating] = (float) dong_nguyen / 100.0;
					if (irms1_arr[Node_IsCommunicating] >= 50)
					{
						irms1_arr[Node_IsCommunicating] = 50;
					}
					/*lay trang thai relay node gui len*/
					RelayStatus_arr_STM[Node_IsCommunicating] = data_node[Node_IsCommunicating][16];
					Check_Uart_update(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
				}
			}

			/*neu ko nhan duoc tin hieu tu node, tang so lan loi va tinh toan trang thai node*/
			if (check_node[Node_IsCommunicating] == 0)
			{
				request_error[Node_IsCommunicating]++;
				if (request_error[Node_IsCommunicating] >= ER_LIMIT_VLUE)
				{
					request_error[Node_IsCommunicating] = ER_LIMIT_VLUE;
					NodeStatus_arr[Node_IsCommunicating] = 0;
				}
			}

			Node_IsCommunicating++;
			if (Node_IsCommunicating >= 10)
			{
				Node_IsCommunicating = 0;
			}
			Send_Request_Lora(Node_IsCommunicating);
			check_node[Node_IsCommunicating] = 0;
			Display_Updated_Flag = true;
			timer_counter++;
			timer_cnt3 = false;
		}

		/*------------ Processing when the button pressed -----------*/
		Touch_XY();

		if(StDisplay == DISPLAY_SCREEN)
		{
			if(Display_Updated_Flag == true)
			{
				// Check if the new value with the old value is different - send to Esp
				stDataChanged = isIrmsChanged(irms1_old, irms1_arr, &index_data_changed[0]);
				if(stDataChanged)
				{
					Update_Current_Val(&index_data_changed[0]);
				}
				Display_Updated_Flag = false;
			}
		}
		if(StDisplay == CTRL_SCREEN_1)
		{
			if(Display_Updated_Flag == true)
			{
				Update_StatusBt(&RelayStatus_arr_STM[0], &RelayStatus_arr_ESP[0]);
				Display_Updated_Flag = false;
			}
		}
		/* Update data after the center circuit has received 10 data frames from the nodes */
		if(timer_counter >= 9)
		{
			uint8_t idx_bf;
			uint8_t idx_relay;
			uint8_t Ctr_Dev_Temp[2]={0, 0};
			uint8_t StNode_Temp[2]={0, 0};

			if(Flag == false)               /* Send only one the first time */
			{
				if(StDisplay == DISPLAY_SCREEN)
				{
					Default_Display();
				}
				for(idx_relay = 0; idx_relay < INDEX_VALUE; idx_relay++)
				{
					if(RelayStatus_arr_STM[idx_relay] == 0)
					{
						CLEAR_BIT_USER(Ctr_Dev, idx_relay);
					}
					else
					{
						SET_BIT_USER(Ctr_Dev, idx_relay);
					}

					if(NodeStatus_arr[idx_relay] == 0)
					{
						CLEAR_BIT_USER(StNode, idx_relay);
					}
					else
					{
						SET_BIT_USER(StNode, idx_relay);
					}
				}

				Send_Uart1(&start_byte_warning[0], &Buzzer, &stop_byte, 2, 1);
				Buzzer_Old = Buzzer;
				/* Send Ctrl_Dev variable */
				Ctr_Dev_Temp[0] = (Ctr_Dev >> 8) & 0xFF;
				Ctr_Dev_Temp[1] = Ctr_Dev & 0xFF;
				Ctr_Dev_Old = Ctr_Dev;
				Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2);
				/* Send node status variable*/
				StNode_Temp[0] = (StNode >> 8) & 0xFF;
				StNode_Temp[1] = StNode & 0xFF;
				StNode_Old = StNode;
				Send_Uart1(&start_byte_status[0], &StNode_Temp[0], &stop_byte, 2, 2);

				/* Send irsm data */
				for (idx_bf = 0; idx_bf < 10; idx_bf++)
				{
					irms1_old[idx_bf] = irms1_arr[idx_bf];    // Copy data to compare with
					ConvertFloatAsBytes(irms1_old[idx_bf], &Tx_Buf[idx_bf * 4]); // Pack 4 bytes per float
				}
				Send_Uart1(&start_byte_data, &Tx_Buf[0], &stop_byte, 1, SIZEOF_DATABUFFER);
				Flag = true;
			}
			else          /* Send only one the continue time */
			{
				Check_Uart_update(&RelayStatus_arr_ESP[0], &RelayStatus_arr_STM[0]);
				for(idx_relay = 0; idx_relay < INDEX_VALUE; idx_relay++)
				{
					if(RelayStatus_arr_STM[idx_relay] == 0)
					{
						CLEAR_BIT_USER(Ctr_Dev, idx_relay);
					}
					else
					{
						SET_BIT_USER(Ctr_Dev, idx_relay);
					}

					if(NodeStatus_arr[idx_relay] == 0)
					{
						CLEAR_BIT_USER(StNode, idx_relay);
					}
					else
					{
						SET_BIT_USER(StNode, idx_relay);
					}
				}
				if(Buzzer_Old != Buzzer)
				{
					Send_Uart1(&start_byte_warning[0], &Buzzer, &stop_byte, 2, 1);
				}

				if(StNode_Old != StNode)
				{
					StNode_Old = StNode;
					StNode_Temp[0] = (StNode_Old >> 8) & 0xFF;
					StNode_Temp[1] = StNode_Old & 0xFF;
					Send_Uart1(&start_byte_status[0], &StNode_Temp[0], &stop_byte, 2, 2);
				}
				if(Ctr_Dev_Old != Ctr_Dev) // Check if the new value with the old value is different - send to Esp
				{
					Ctr_Dev_Old = Ctr_Dev;
					Ctr_Dev_Temp[0] = (Ctr_Dev_Old >> 8) & 0xFF;
					Ctr_Dev_Temp[1] = Ctr_Dev_Old & 0xFF;
					Send_Uart1(&start_byte_ctrl[0], &Ctr_Dev_Temp[0], &stop_byte, 2, 2);
				}

				if(stDataChanged == true) // Check if the new value with the old value is different - send to Esp
				{
					for (idx_bf = 0; idx_bf < 10; idx_bf++)
					{
						ConvertFloatAsBytes(irms1_old[idx_bf], &Tx_Buf[idx_bf * 4]); // Pack 4 bytes per float
					}
					Send_Uart1(&start_byte_data, &Tx_Buf[0], &stop_byte, 1, SIZEOF_DATABUFFER);
				}
			}
			timer_counter = 0;
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TP_CS_Pin|TFT_DC_Pin|TFT_RST_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP_CS_Pin TFT_DC_Pin TFT_RST_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TP_CS_Pin|TFT_DC_Pin|TFT_RST_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
