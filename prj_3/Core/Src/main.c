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
#include "uart.h"
#include "filter.h"
#include <stdio.h>
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delayUs(uint16_t time) {
	htim10.Instance->CNT = 0;
	while (htim10.Instance->CNT < time)
		;
}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t IC_Val3 = 0;
uint32_t IC_Val4 = 0;
uint32_t IC_Val5 = 0;
uint32_t IC_Val6 = 0;
uint32_t Difference1 = 0;
uint8_t IsFirstCaptured1 = 0;
uint32_t Distance1 = 0;
uint32_t Difference3 = 0;
uint8_t IsFirstCaptured3 = 0;
uint32_t Distance3 = 0;
uint32_t Difference4 = 0;
uint8_t IsFirstCaptured4 = 0;
uint32_t Distance4 = 0;
uint8_t flagState = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (IsFirstCaptured1 == 0 && flagState == 0) {
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			IsFirstCaptured1 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (IsFirstCaptured1 == 1) {
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			htim->Instance->CNT = 0;
			if (IC_Val2 > IC_Val1)
				Difference1 = IC_Val2 - IC_Val1;
			else if (IC_Val1 > IC_Val2)
				Difference1 = (0xffff - IC_Val1) + IC_Val2;
			Distance1 = Difference1 * 0.034 / 2;
			IsFirstCaptured1 = 0;
			flagState = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (IsFirstCaptured3 == 0 && flagState == 1) {
			IC_Val3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			IsFirstCaptured3 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (IsFirstCaptured3 == 1) {
			IC_Val4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			htim->Instance->CNT = 0;
			if (IC_Val4 > IC_Val3)
				Difference3 = IC_Val4 - IC_Val3;
			else if (IC_Val3 > IC_Val4)
				Difference3 = (0xffff - IC_Val3) + IC_Val4;
			Distance3 = Difference3 * 0.034 / 2;
			IsFirstCaptured3 = 0;
			flagState = 2;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (IsFirstCaptured4 == 0 && flagState == 2) {
			IC_Val5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			IsFirstCaptured4 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (IsFirstCaptured4 == 1) {
			IC_Val6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			htim->Instance->CNT = 0;
			if (IC_Val6 > IC_Val5)
				Difference4 = IC_Val6 - IC_Val5;
			else if (IC_Val5 > IC_Val6)
				Difference4 = (0xffff - IC_Val5) + IC_Val6;
			Distance4 = Difference4 * 0.034 / 2;
			IsFirstCaptured4 = 0;
			flagState = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
		}
	}
}

uint32_t getDistance1() {
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 1);
	delayUs(10);
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 0);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	return Distance1;
}
uint32_t getDistance3() {
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 1);
	delayUs(10);
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 0);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
	return Distance3;
}
uint32_t getDistance4() {
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 1);
	delayUs(10);
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 0);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
	return Distance4;
}

int soundLUT[] = { 130, 146, 164, 174, 196, 220, 246, 261 };

void setSound(int freq) {
	htim3.Instance->ARR = 1000000 / freq - 1;
	htim3.Instance->CCR1 = htim3.Instance->ARR / 2;
}
void stopSound() {
	htim3.Instance->CCR1 = 0;
}

typedef struct ultraData {
	uint16_t ultraCH_1;
	uint16_t ultraCH_3;
	uint16_t ultraCH_4;
} ultraData_t;

ultraData_t getDistance() {
	ultraData_t dist;
	dist.ultraCH_1 = getDistance1();
	dist.ultraCH_3 = getDistance3();
	dist.ultraCH_4 = getDistance4();

	return dist;
}

void motorActive() {
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	htim2.Instance->CCR1 = 500;
}
void motorStop() {
	htim2.Instance->CCR1 = 0;
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
}
void ledOff() {
	HAL_GPIO_WritePin(TEST_RED_GPIO_Port, TEST_RED_Pin, 0);
	HAL_GPIO_WritePin(TEST_GREEN_GPIO_Port, TEST_GREEN_Pin, 0);
	HAL_GPIO_WritePin(TEST_BLUE_GPIO_Port, TEST_BLUE_Pin, 0);
}
void ledRedOn() {
	HAL_GPIO_WritePin(TEST_RED_GPIO_Port, TEST_RED_Pin, 1);
}
void ledGreenOn() {
	HAL_GPIO_WritePin(TEST_GREEN_GPIO_Port, TEST_GREEN_Pin, 1);
}
void ledBlueOn() {
	HAL_GPIO_WritePin(TEST_BLUE_GPIO_Port, TEST_BLUE_Pin, 1);
}

uint8_t BCD2Decimal(uint8_t inData) {
	uint8_t upper = inData >> 4;
	uint8_t lower = inData & 0x0f;
	return upper * 10 + lower;
}

uint8_t Decimal2BCD(uint8_t inData) {
	uint8_t upper = inData / 10;
	uint8_t lower = inData % 10;
	return upper << 4 | lower;
}

#define RTC_ADD          0xD0
#define ROM_ADD          0xA0
#define READ               1
#define MagicNumber      0x257fad2e      // random number

// EEPROM Map Table
#define eeMagicNumberBase      0
#define eeMagicNumberSize      4      // 4byte

typedef struct {
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
} DateTime_t;

void setRTC(DateTime_t inData) {
	uint8_t txBuffer[8];
	txBuffer[7] = Decimal2BCD(inData.year);
	txBuffer[6] = Decimal2BCD(inData.month);
	txBuffer[5] = Decimal2BCD(inData.date);
	txBuffer[3] = Decimal2BCD(inData.hour);
	txBuffer[2] = Decimal2BCD(inData.min);
	txBuffer[1] = Decimal2BCD(inData.sec);
	txBuffer[0] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, RTC_ADD, txBuffer, sizeof(txBuffer), 10);
}

DateTime_t getRTC() {
	DateTime_t result;
	uint8_t rxBuffer[7];
	uint8_t address = 0;
	HAL_I2C_Master_Transmit(&hi2c1, RTC_ADD, &address, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, RTC_ADD | READ, rxBuffer, 7, 10);
	result.year = BCD2Decimal(rxBuffer[6]);
	result.month = BCD2Decimal(rxBuffer[5]);
	result.date = BCD2Decimal(rxBuffer[4]);
	result.day = BCD2Decimal(rxBuffer[3]);
	result.hour = BCD2Decimal(rxBuffer[2]);
	result.min = BCD2Decimal(rxBuffer[1]);
	result.sec = BCD2Decimal(rxBuffer[0]);
	return result;
}

// address : 0 ~ 56 (0x00 ~ 0x37)
/*void writeRAM(uint8_t address, uint8_t data) {
 uint8_t txBuffer[2];
 txBuffer[0] = address - 8;
 txBuffer[1] = data;
 HAL_I2C_Master_Transmit(&hi2c1, RTC_ADD, txBuffer, sizeof(txBuffer), 10);
 }
 */
void writeRAM(uint8_t address, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c1, RTC_ADD, address, 1, &data, 1, 10);
}
/*
 uint8_t readRAM(uint8_t address) {
 uint8_t result;
 uint8_t address2 = address - 8;
 HAL_I2C_Master_Transmit(&hi2c1, RTC_ADD, &address2, 1, 10);
 HAL_I2C_Master_Receive(&hi2c1, RTC_ADD | READ, &result, 1, 10);
 }*/
uint8_t readRAM(uint8_t address) {
	uint8_t result;
	HAL_I2C_Mem_Read(&hi2c1, RTC_ADD, address, 1, &result, 1, 10);
	return result;
}

void writeEEPROM(uint16_t address, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c1, ROM_ADD, address, 2, &data, 1, 10);
	HAL_Delay(5);   // data reading delay time : 5ms
}

uint8_t readEEPROM(uint16_t address) {
	uint8_t result;
	HAL_I2C_Mem_Read(&hi2c1, ROM_ADD, address, 2, &result, 1, 10);
	return result;
}

void write2ByteEEPROM(uint16_t address, uint16_t data) {
	HAL_I2C_Mem_Write(&hi2c1, ROM_ADD, address, 2, &data, 2, 10);
	HAL_Delay(10);
}

uint16_t read2ByteEEPROM(uint16_t address) {
	uint16_t result;
	HAL_I2C_Mem_Read(&hi2c1, ROM_ADD, address, 2, &result, 2, 10);
	return result;
}

void write4ByteEEPROM(uint16_t address, uint32_t data) {
	HAL_I2C_Mem_Write(&hi2c1, ROM_ADD, address, 2, &data, 4, 10);
	HAL_Delay(20);
}

uint32_t read4ByteEEPROM(uint16_t address) {
	uint32_t result;
	HAL_I2C_Mem_Read(&hi2c1, ROM_ADD, address, 2, &result, 4, 10);
	return result;
}

int countRTC = 0;
int countLCD_Blink = 0;
int countArrow = 0;
int countMotor = 0;

void SysTickCallback() {      // 1mS마다 call
	if (countRTC > 0)
		countRTC--;
	if (countLCD_Blink > 0)
		countLCD_Blink--;
	if (countArrow > 0)
		countArrow--;
	if (countMotor > 0)
		countMotor--;
}
void LCD_BLINK() {
	ILI9341_FillScreen2(BLACK);
	//if(countLCD_Blink == 0)
	//{
	//   countLCD_Blink = 100;
	//}

}

void ILI9341_FillScreen2(uint16_t color) {
	ILI9341_SetAddress(0, 69, LCD_WIDTH, LCD_HEIGHT);
	ILI9341_DrawColorBurst(color, LCD_WIDTH * 251);
}

void LCD_DrawArrow(uint8_t num) {
	switch (num) {
	case 0:
		ILI9341_FillScreen2(BLACK);
		ILI9341_DrawFilledRectangleCoord(110, 280, 130, 320, GREEN);
		for (int j = 15; j > 0; j--) {
			ILI9341_DrawFilledRectangleCoord(90 + 2 * j, 282 - 2 * j,
					150 - 2 * j, 280 - 2 * j, GREEN);
			// 180-210
		}
		break;

	case 1:
		ILI9341_FillScreen2(BLACK);
		ILI9341_DrawFilledRectangleCoord(110, 170, 130, 210, GREEN);
		for (int j = 15; j > 0; j--) {
			ILI9341_DrawFilledRectangleCoord(90 + 2 * j, 172 - 2 * j,
					150 - 2 * j, 170 - 2 * j, GREEN);
			// 180-210
		}
		break;
	case 2:
		ILI9341_FillScreen2(BLACK);
		ILI9341_DrawFilledRectangleCoord(110, 100, 130, 140, GREEN);
		for (int j = 15; j > 0; j--) {
			ILI9341_DrawFilledRectangleCoord(90 + 2 * j, 102 - 2 * j,
					150 - 2 * j, 100 - 2 * j, GREEN);
			// 180-210
		}
		break;
	}

}
void LCD_DrawStopSign() {

	ILI9341_FillScreen2(BLACK);
	ILI9341_DrawFilledRectangleCoord(90, 165, 150, 175, RED);
	ILI9341_DrawHollowCircle(120, 170, 60, RED);
	ILI9341_DrawHollowCircle(120, 170, 58, RED);
	ILI9341_DrawHollowCircle(120, 170, 59, RED);
	ILI9341_DrawHollowCircle(120, 170, 55, RED);
	ILI9341_DrawHollowCircle(120, 170, 54, RED);
	ILI9341_DrawHollowCircle(120, 170, 56, RED);
	ILI9341_DrawHollowCircle(120, 170, 57, RED);

}

uint8_t motorState = 0;

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	initUART(&huart2);
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	DateTime_t dateTime;
	dateTime.year = 24;
	dateTime.month = 4;
	dateTime.date = 12;
	dateTime.hour = 16;
	dateTime.min = 43;
	dateTime.sec = 0;
	setRTC(dateTime);

	uint8_t num = 0;

	uint8_t runState = 0;

	enum {
		STOP, STANDBY, RUN1, RUN2, RUN3
	};

	//initUart(&huart2);

	// set up color lcd
	ILI9341_Init();
	ILI9341_SetRotation(SCREEN_VERTICAL_1);
	ILI9341_FillScreen(BLACK);

	// RTC data R/T
	uint8_t address = 0;
	uint8_t buffer[0x40] = { 0, };
	HAL_I2C_Master_Transmit(&hi2c1, 0xd0, &address, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, 0xd1, buffer, 1, 10);
	buffer[1] = buffer[0] & 0x7f;
	buffer[0] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, 0xd0, buffer, 2, 10);

	stopSound();
	htim4.Instance->CCR1 = 600;
	//LCD_DrawStopSign();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		static uint8_t waitCount;
		ultraData_t ultraData = getDistance();
		//uint16_t CH_value1 = getDistance1();
		//uint16_t CH_value2 = getDistance3();
		//uint16_t CH_value3 = getDistance4();
		//printf("%d %d %d\n", CH_value1, CH_value2, CH_value3);
		printf("%d %d %d\n", ultraData.ultraCH_1, ultraData.ultraCH_3,
				ultraData.ultraCH_4);
		if (ultraData.ultraCH_1 > 10 && ultraData.ultraCH_3 < 5) {
			ledGreenOn();
			HAL_Delay(500);
			ledOff();
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			htim2.Instance->CCR1 = 500;
			htim4.Instance->CCR1 = 600;
			stopSound();

		} else if (ultraData.ultraCH_3 > 10 && ultraData.ultraCH_1 < 3) {
			ledRedOn();
			motorStop();
			LCD_DrawStopSign();
			htim4.Instance->CCR1 = 1500;
			setSound(soundLUT[0]);
			HAL_Delay(5000);
			ledOff();
		} else if (waitCount > 1500) {
			ultraData = getDistance();
			if (ultraData.ultraCH_4 > 10)
				waitCount = 0;

		} else {
			motorActive();
			htim4.Instance->CCR1 = 600;
			stopSound();
			waitCount++;
		}
		HAL_Delay(50);
		/*uint16_t servovalue = 0;
		 servovalue = 600;
		 htim4.Instance->CCR1 = servovalue;
		 printf("%d\n", htim4.Instance->CCR1);
		 HAL_Delay(1000);

		 servovalue = 1500;
		 htim4.Instance->CCR1 = servovalue;
		 printf("%d\n", htim4.Instance->CCR1);
		 HAL_Delay(1000);*/
		if (countRTC == 0) {
			countRTC = 1000;
			dateTime = getRTC();
			char str1[30];
			char str2[30];
			sprintf(str1, "%02d-%02d-%02d", dateTime.year, dateTime.month,
					dateTime.date);
			sprintf(str2, "%02d:%02d:%02d", dateTime.hour, dateTime.min,
					dateTime.sec);

			ILI9341_DrawText(str1, FONT4, 85, 30, WHITE, BLACK);
			ILI9341_DrawText(str2, FONT4, 86, 50, WHITE, BLACK);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin|IN2_Pin
                          |IN1_Pin|TEST_GREEN_Pin|TEST_RED_Pin|TEST_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_CS_Pin LCD_DC_Pin LCD_RST_Pin IN2_Pin
                           IN1_Pin TEST_GREEN_Pin TEST_RED_Pin TEST_BLUE_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin|IN2_Pin
                          |IN1_Pin|TEST_GREEN_Pin|TEST_RED_Pin|TEST_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RESTART_btn_Pin EM_btn_Pin */
  GPIO_InitStruct.Pin = RESTART_btn_Pin|EM_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : trigger_Pin */
  GPIO_InitStruct.Pin = trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trigger_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EM_btn_Pin) {
		motorStop();
		ledBlueOn();

	}
	if (GPIO_Pin == RESTART_btn_Pin) {
		motorActive();
		ledOff();
	}
}
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
