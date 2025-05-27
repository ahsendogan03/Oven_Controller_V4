/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "SEGGER_RTT.h"
#include "string.h"
#include "stdio.h"
#include "DWIN_Process.h"
#include "Temperature_Process.h"
#include "InOut_Process.h"
#include "USART_Process.h"
#include "EEPROM_Process.h"
#include "Bluetooth_Process.h"
//#include "sht40.h"
//#include "hdc1080.h"
#include "TMP112.h"
#include "PID_Control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint16_t DWIN_version;
extern usartInfo DWIN;
extern usartInfo ESP32;
extern uint32_t outputData;
extern EEPROM_initResponse eepromStatus;
extern TMP112 tmpSensor;
//extern HDC1080 hdcSensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint16_t registerTable[9000];
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  ShiftRegister_SendData(0);
  HAL_Delay(0);
  SEGGER_RTT_ConfigUpBuffer(0,NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  HAL_Delay(0);
  SEGGER_RTT_printf(0,"-------- SYSTEM START -------- \r\n");

	  for(int i=0;i<6;i++)
	  {
		  HAL_Delay(30);
		  setOut(BUZZER, HIGH);
		  HAL_Delay(30);
		  setOut(BUZZER, LOW);

		  HAL_GPIO_TogglePin(RUN_LED);
	  }

	  if(adc_Init() != HAL_OK)
	  {
		  SEGGER_RTT_printf(0,"ADC Error \r\n");
		  Error_Handler();
	  }

	  if(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_BASE_ADDR << 1, 2, 100) != HAL_OK)
	  {
		  eepromStatus = EE_INIT_ERROR;
		  SEGGER_RTT_printf(0,"EEPROM Error \r\n");
	  }

	  if(DWIN_SetUsartChannel(&huart3, USART3, &hdma_usart3_rx) != HAL_OK)
	  {
		  SEGGER_RTT_printf(0,"DWIN Set Usart Channel Error ! \r\n");
		  Error_Handler();
	  }

	  if(ESP32_SetUsartChannel(&huart1, USART1, &hdma_usart1_rx) != HAL_OK)
	  {
		  SEGGER_RTT_printf(0,"ESP32 Set Usart Channel Error ! \r\n");
		  Error_Handler();
	  }

	//  sht4x_Init(&hi2c1);
	//  sht4x_setMode(SHT4x_NOHEAT_LOWPRECISION);

//	  if (HDC1080_Init(&hdcSensor, &hi2c1,
//	                   HDC1080_RESOLUTION_TEMP_14BIT,
//	                   HDC1080_RESOLUTION_HUM_14BIT) == HAL_OK)
//	  {
//		  SEGGER_RTT_printf(0,"HDC1080 Init OK ! \r\n");
//	  }

//	  else
//	  {
//		  SEGGER_RTT_printf(0,"HDC1080 Init ERROR ! \r\n");
//	  }

	  if(TMP112_Init(&tmpSensor, &hi2c1) == HAL_OK)
	  {
		  // 10-bit çözünürlük ayarla
		  TMP112_SetResolution(&tmpSensor, TMP112_RESOLUTION_12_BIT);

		  // Shutdown modunu etkinleştir
		  //TMP112_EnableShutdownMode(&mySensor, 1);

		  // Tek seferlik ölçüm başlat
		  //TMP112_TriggerOneShot(&mySensor);
	  }

	  else
	  {
		  SEGGER_RTT_printf(0,"TMP112 Init ERROR ! \r\n");
	  }

	  memset(registerTable,0,sizeof(registerTable));

	  RTC_SetDateTime(15, 30, 0, 27, 1, 25);

	  if(EEPROM_init(&hi2c1) != EE_INIT_OK)
		  SEGGER_RTT_printf(0,"EEPROM Init Error ! \r\n");


	  HAL_Delay(100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  DWIN_run();
	  Bluetooth_run();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
