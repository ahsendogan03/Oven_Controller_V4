/*
 * Temperature_Process.h
 *
 *  Created on: Jan 20, 2025
 *      Author: Step
 */

#ifndef INC_TEMPERATURE_PROCESS_H_
#define INC_TEMPERATURE_PROCESS_H_

#include "main.h"

#define DEBUG_Temp 0

#define VREF_ROW_BUFFER 	3
//#define MCP9700_ROW_BUFFER 	0
#define TC1_ROW_BUFFER 		0
#define TC2_ROW_BUFFER 		1
#define TC3_ROW_BUFFER 		2

#define TEMP_LOG_CHECK_ADR		99
#define TEMP_LOG_START_ADR		8500
#define TEMP_LOG_START_ADR_2	10500



#define numOfChannel 4
#define numOfSample 200

#define VREFINT_CAL_VAL	 1535  // STM32'deki VREFINT kalibrasyon adresi
#define ADC_RESOLUTION   4096  // 12-bit ADC (STM32 için tipik)
#define VREF_CAL_VALUE   3237  // Fabrika kalibrasyonu varsayılan olarak 3.0V üzerinden yapılır (mV)

typedef struct  {
    int TC1;
    int TC2;
    int TC3;
    int MCP9700;
    int SHT40;
    int HDC;
    int TMP;
}TemperatureData;

int calculate_termocouple(uint16_t adc_value);
HAL_StatusTypeDef adc_Init(void);
void avgAdcProcess(void);
void calculate_temperature(void);

#endif /* INC_TEMPERATURE_PROCESS_H_ */
