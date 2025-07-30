/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include "SEGGER_RTT.h"

// Ayların gün sayısı (Şubat ayı için artık yıl kontrolü yapılır)
const uint8_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// Artık yıl kontrolü
uint8_t isLeapYear(uint16_t year) {
    return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
}

typedef struct {
    uint32_t RemainingMinutes;
    uint32_t RemainingSeconds;
} CountdownTimer;

// İstenilen saniye sonrasını hesaplayan fonksiyon
void CalculateFutureTime(RTC_DateTypeDef *currentDate, RTC_TimeTypeDef *currentTime,
                         RTC_DateTypeDef *futureDate, RTC_TimeTypeDef *futureTime,
                         uint32_t offsetSeconds) {
    // Geçerli tarih ve saati kopyala
    *futureDate = *currentDate;
    *futureTime = *currentTime;

    // Toplam saniyeyi hesapla
    uint32_t totalSeconds = futureTime->Seconds + offsetSeconds;

    // Saniyeleri dakikaya dönüştür
    futureTime->Seconds = totalSeconds % 60;
    uint32_t totalMinutes = futureTime->Minutes + (totalSeconds / 60);

    // Dakikaları saate dönüştür
    futureTime->Minutes = totalMinutes % 60;
    uint32_t totalHours = futureTime->Hours + (totalMinutes / 60);

    // Saatleri güne dönüştür
    futureTime->Hours = totalHours % 24;
    uint32_t totalDays = totalHours / 24;

    // Tarihi güncellemeye başla
    while (totalDays > 0) {
        uint8_t maxDays = daysInMonth[futureDate->Month - 1];

        // Şubat için artık yıl kontrolü
        if (futureDate->Month == 2 && isLeapYear(2000 + futureDate->Year)) {
            maxDays = 29;
        }

        if (futureDate->Date + totalDays > maxDays) {
            totalDays -= (maxDays - futureDate->Date + 1);
            futureDate->Date = 1;
            futureDate->Month++;

            // Ay taşması (Aralık'tan Ocak'a geçiş)
            if (futureDate->Month > 12) {
                futureDate->Month = 1;
                futureDate->Year++;
            }
        } else {
            futureDate->Date += totalDays;
            totalDays = 0;
        }
    }
}


void RTC_SetDateTime(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t month, uint16_t year) {

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // Saat ayarla
    sTime.Hours 	= hour;
    sTime.Minutes 	= minute;
    sTime.Seconds 	= second;


    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        // Hata durumunda buraya gelir
        Error_Handler();
    }

    // Tarih ayarla
    sDate.Month 	= month;                	// Ay
    sDate.Date 		= day;                   	// Gün
    sDate.Year 		= year;           			// Yıl (örneğin 2025 için 25)

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        // Hata durumunda buraya gelir
        Error_Handler();
    }
}


void RTC_GetDateTime(RTC_TimeTypeDef *Time, RTC_DateTypeDef	*Date)
{

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};


    //Saat bilgisi oku
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // Tarih bilgisi oku
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    Time->Hours 	= sTime.Hours;
    Time->Minutes 	= sTime.Minutes;
    Time->Seconds	= sTime.Seconds;

    Date->Date		= sDate.Date;
    Date->Month		= sDate.Month;
    Date->WeekDay	= sDate.WeekDay;
    Date->Year		= sDate.Year;

//    SEGGER_RTT_printf(0,"Saat: %02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
//    SEGGER_RTT_printf(0,"Tarih: %02d/%02d/20%02d  gun:%d\n", sDate.Date, sDate.Month, sDate.Year,sDate.WeekDay);

}
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
