/*
 * Temperature_Process.c
 *
 *  Created on: Jan 20, 2025
 *      Author: Step
 */

// Test Commit

#include "Temperature_Process.h"
#include "SEGGER_RTT.h"

#include "DWIN_Process.h"
#include "TMP112.h"
#include "math.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

uint8_t adcInitFlag = 0;

uint32_t adcBuffer[numOfChannel];
uint32_t tempBuffer[numOfChannel];
uint32_t sumAdcBuffer[numOfChannel];
uint32_t avgAdcBuffer[numOfChannel];
uint32_t avgCounter = 0;

uint16_t vref;

TemperatureData temp;
TMP112 tmpSensor;


extern uint16_t registerTable[REGISTER_TABLE_SIZE];

uint16_t templog_amount = 0;


uint16_t calculate_vref(uint16_t adc_vrefint) {
    // VREFINT için kalibrasyon değerini okuyoruz
    uint16_t vrefint_cal = VREFINT_CAL_VAL;

    // Gerçek referans voltajını hesaplıyoruz (mV cinsinden)
    float vref_actual = (float)vrefint_cal * VREF_CAL_VALUE / adc_vrefint;

    // mV'den Volt'a dönüştürüyoruz
    return vref_actual;
}


HAL_StatusTypeDef adc_Init(void)
{
	HAL_StatusTypeDef response = HAL_ERROR;

	if(HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK)
	{
		response = HAL_ADC_Start_DMA(&hadc1,adcBuffer,numOfChannel);
	}

	if(response == HAL_OK)
	{
		HAL_Delay(0);
		adcInitFlag = 1;
	}

	return response;

}

int ustOnSicaklik 		= 0;
int ustArkaSicaklik 	= 0;
int altSicaklik 		= 0;

void avgAdcProcess(void) // ms period function
{
	if(adcInitFlag)
	{

		for(int j=0;j<numOfChannel;j++)
		{
			sumAdcBuffer[j] += adcBuffer[j];
		}

		avgCounter++;

		if(avgCounter >= numOfSample)
		{
			for(int j=0;j<numOfChannel;j++)
			{
				avgAdcBuffer[j] = sumAdcBuffer[j] / numOfSample;
				sumAdcBuffer[j] = 0;
			}

			avgCounter = 0;
		}

	}
}

#define ADC_OFFSET_K      510
//#define ADC_PER_DEGREE  4.63f  //610 5.2 2100 4.65

float ConvertValueforKType(uint16_t input)
{
    if (input <= 550)
        return 5.10f;

    if (input >= 2100)
        return 4.69f;

    float t = (float)(input - 550) / (2100.0f - 550.0f); // 0..1

    // %10 daha agresif ease-out
    float curved = 1.0f - powf((1.0f - t), 2.2f);

    return 5.10f + curved * (4.69f - 5.10f);
}

int calculateThermocouple_K(uint16_t adc_value) {
    // 1. Adım: ADC değerinden sanal sıfır noktasını çıkar (Negatif olabilir)
    int32_t adc_difference = (int32_t)adc_value - ADC_OFFSET_K;

    float ADC_PER_DEGREE = ConvertValueforKType(adc_value);

    // 2. Adım: ADC farkını sıcaklık farkına çevir
    // float işlemi ile hassas bölme yapıp int'e çeviriyoruz.
    int32_t delta_temp = (int32_t)(adc_difference / ADC_PER_DEGREE);

    // 3. Adım: Ortam sıcaklığını (Cold Junction) ekle
    int result_temp = delta_temp + temp.TMP;

    return result_temp;
}

#define ADC_OFFSET_J      515

float ConvertValueforJType(uint16_t input)
{
    if (input <= 550)
        return 5.80f;     // artık düşükte küçük

    if (input >= 2500)
        return 6.30f;     // yüksekte büyük

    float t = (float)(input - 550) / (2500.0f - 550.0f); // 0..1

    float curved = 1.0f - powf((1.0f - t), 1.5f);

    return 5.80f + curved * (6.30f - 5.80f);
}

int calculateThermocouple_J(uint16_t adc_value) {
    // 1. Adım: ADC değerinden sanal sıfır noktasını çıkar (Negatif olabilir)
    int32_t adc_difference = (int32_t)adc_value - ADC_OFFSET_J;

    float ADC_PER_DEGREE = ConvertValueforJType(adc_value);

    // 2. Adım: ADC farkını sıcaklık farkına çevir
    // float işlemi ile hassas bölme yapıp int'e çeviriyoruz.
    int32_t delta_temp = (int32_t)(adc_difference / ADC_PER_DEGREE);

    // 3. Adım: Ortam sıcaklığını (Cold Junction) ekle
    int result_temp = delta_temp + temp.TMP;

    return result_temp;
}

uint8_t intTempSampleCounter = 0;
float tmp_temp;
void calculate_temperature(void)
{

	if(intTempSampleCounter == 0)
	{
		if(TMP112_ReadTemperature(&tmpSensor, &tmp_temp) == HAL_OK)
			temp.TMP = tmp_temp;
		else
			SEGGER_RTT_printf(0," TMP112 Read Temp ERROR ! \r\n");
	}

	intTempSampleCounter++;

	if(intTempSampleCounter >= 5)
		intTempSampleCounter = 0;

	vref 			= calculate_vref(avgAdcBuffer[VREF_ROW_BUFFER]);

	if(registerTable[DW_PARAM_TERMOKUPL_TYPE_ADR] == DW_K_TYPE_TERMOKUP_VAL)
	{
		temp.TC1 		= calculateThermocouple_K(avgAdcBuffer[TC1_ROW_BUFFER]);
		temp.TC2 		= calculateThermocouple_K(avgAdcBuffer[TC2_ROW_BUFFER]);
		temp.TC3 		= calculateThermocouple_K(avgAdcBuffer[TC3_ROW_BUFFER]);
	}

	else if(registerTable[DW_PARAM_TERMOKUPL_TYPE_ADR] == DW_J_TYPE_TERMOKUP_VAL)
	{
		temp.TC1 		= calculateThermocouple_J(avgAdcBuffer[TC1_ROW_BUFFER]);
		temp.TC2 		= calculateThermocouple_J(avgAdcBuffer[TC2_ROW_BUFFER]);
		temp.TC3 		= calculateThermocouple_J(avgAdcBuffer[TC3_ROW_BUFFER]);
	}


	ustOnSicaklik 	= temp.TC1;
	ustArkaSicaklik = temp.TC3;
	altSicaklik 	= temp.TC2;


	#if DEBUG_Temp == 1
	SEGGER_RTT_printf(0,"--------------------------------------------------------- \r\n");
	SEGGER_RTT_printf(0,"VREF_mv :  %d  TMP112 :     %d    TC1 Temp : %d   TC2 Temp : %d   TC3 Temp : %d \r\n",
						vref, temp.MCP9700, temp.TC1, temp.TC2, temp.TC3);

	SEGGER_RTT_printf(0,"VREF Val : %d  TMP112 : %d  TC1 Val  : %d  TC2 Val  : %d  TC3 Val  : %d \r\n",
						avgAdcBuffer[VREF_ROW_BUFFER],avgAdcBuffer[MCP9700_ROW_BUFFER],avgAdcBuffer[TC1_ROW_BUFFER],avgAdcBuffer[TC2_ROW_BUFFER],avgAdcBuffer[TC3_ROW_BUFFER]);

	#endif



}
