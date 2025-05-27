/*
 * sht40.c
 *
 *  Created on: Mar 14, 2025
 *      Author: Step
 */


#include "sht40.h"
#include "i2c.h"
#include "SEGGER_RTT.h"
#include "string.h"
#include "math.h"



struct tempHum{

	uint8_t adress;
	uint8_t rxBuffer[6];
	uint8_t initCheck;
	uint16_t waitMs;
	uint8_t operatingMode;
	uint8_t errorCounter;
	I2C_HandleTypeDef *hi2c;

}sht4x;



uint8_t sht4x_Init(I2C_HandleTypeDef *handle)
{
	struct sht4x;
	sht4x.hi2c = handle;
	sht4x.initCheck = 0;

	if(HAL_OK != HAL_I2C_IsDeviceReady(sht4x.hi2c, SHT4x_DEFAULT_ADDR << 1, 2, 100))
	{
//		for(int i=0x01; i<=0xFF; i++)
//		{
//			HAL_Delay(0);
//
//			if(HAL_OK == HAL_I2C_IsDeviceReady(sht4x.hi2c, i << 1, 1, 10))
//			{
//				sht4x.adress = i;
//				SEGGER_RTT_printf(0,"SHT40 Adress :");SEGGER_RTT_printf(0,"0x%x \r\n",i);
//				sht4x.initCheck = Init_OK;
//				break;
//			}
//
//		}
//		if(sht4x.initCheck != Init_OK)
//		{
			SEGGER_RTT_printf(0,"SHT40 Not Found ! \r\n");
			return 1;
//		}
	}
	else
	{
		sht4x.initCheck = Init_OK;
		sht4x.adress = SHT4x_DEFAULT_ADDR;
		SEGGER_RTT_printf(0,"SHT40 Adress :");SEGGER_RTT_printf(0,"0x%x \r\n",SHT4x_DEFAULT_ADDR);
	}

	sht4x.operatingMode = SHT4x_NOHEAT_LOWPRECISION;
	return 0;
}
uint8_t i2c_read(uint8_t cmd)
{
	if(sht4x.initCheck == Init_OK)
	{
		switch(sht4x.operatingMode)
		{
		case SHT4x_NOHEAT_HIGHPRECISION:
			sht4x.waitMs = 9;
			break;
		case SHT4x_NOHEAT_MEDPRECISION:
			sht4x.waitMs = 5;
			break;
		case SHT4x_NOHEAT_LOWPRECISION:
			sht4x.waitMs = 2;
			break;
		case SHT4x_HIGHHEAT_1S:
			sht4x.waitMs = 1100;
			break;
		case SHT4x_HIGHHEAT_100MS:
			sht4x.waitMs = 110;
			break;
		case SHT4x_MEDHEAT_1S:
			sht4x.waitMs = 1100;
			break;
		case SHT4x_MEDHEAT_100MS:
			sht4x.waitMs = 110;
			break;
		case SHT4x_LOWHEAT_1S:
			sht4x.waitMs = 1100;
			break;
		case SHT4x_LOWHEAT_100MS:
			sht4x.waitMs = 110;
			break;
		default:
			sht4x.waitMs = 110;
			break;
		}

		HAL_StatusTypeDef status;

		errorCheckProcess();

		if(cmd == SHT4x_SOFTRESET)
		{
			status = HAL_I2C_Master_Transmit(sht4x.hi2c, sht4x.adress << 1, &cmd, 1, 10);
		}
		else
		{

			HAL_I2C_Master_Transmit(sht4x.hi2c, sht4x.adress << 1, &cmd, 1, 10);
			HAL_Delay(sht4x.waitMs);
			status = HAL_I2C_Master_Receive(sht4x.hi2c, sht4x.adress << 1, sht4x.rxBuffer, 6, 10);
		}

		switch(status)
		{

		case HAL_OK:
			sht4x.errorCounter = 0;
			break;

		case HAL_BUSY:
			sht4x.errorCounter++;
			SEGGER_RTT_printf(0,"BUSY !\r\n");
			return HAL_BUSY;
			break;

		case HAL_TIMEOUT:
			sht4x.errorCounter++;
			SEGGER_RTT_printf(0,"TIMEOUT !\r\n");
			return HAL_TIMEOUT;
			break;

		case HAL_ERROR:
			sht4x.errorCounter++;
			SEGGER_RTT_printf(0,"ERROR !\r\n");
			return HAL_ERROR;
			break;
		}
	}
	else
	{
		SEGGER_RTT_printf(0,"SHT40 initialize error ! \r\n");
		SEGGER_RTT_printf(0,"Please sensor initialize again ! \r\n");
		return 1;
	}

	return HAL_OK;
}

void sht4x_setMode(sht4x_mode_t cmd)
{
	sht4x.operatingMode = cmd;
}

uint8_t sht4xGetEvent(float *pTemp, float *pHum)
{
	HAL_StatusTypeDef status;

	status = i2c_read(sht4x.operatingMode);
	if(status == HAL_OK)
	{
		if (sht4x.rxBuffer[2] != crc8(sht4x.rxBuffer, 2) || sht4x.rxBuffer[5] != crc8(sht4x.rxBuffer + 3, 2))
		{
			SEGGER_RTT_printf(0,"CRC Error ! \r\n");
			sht4x.errorCounter++;
			return CRC_ERR;
		}

		else
		{
			float temprature,humidity;
			float t_ticks = (sht4x.rxBuffer[0] * 256) + sht4x.rxBuffer[1];
			float rh_ticks = (sht4x.rxBuffer[3] * 256) + sht4x.rxBuffer[4];
			temprature = -45 + 175 * t_ticks / 65535;
			humidity = -6 + 125 * rh_ticks / 65535;
			humidity = fmin(fmax(humidity, (float)0.0), (float)100.0);

			*pTemp = temprature;
			*pHum = humidity;

			memset(sht4x.rxBuffer,0,sizeof(sht4x.rxBuffer));
		}
	}
	else
	{
		return status;
	}

	return 0;
}
uint8_t sht4xGetHeat(sht4x_heat_t cmd, float *pTemp, float *pHum)
{
	HAL_StatusTypeDef status;
	uint8_t modeData = sht4x.operatingMode;
	sht4x.operatingMode = cmd;
	status = sht4xGetEvent(pTemp, pHum);
	sht4x.operatingMode = modeData;

	return status;
}
void errorCheckProcess()
{
	if(sht4x.errorCounter >= 3)
	{
		sht4x.errorCounter = 0;

		if (HAL_I2C_DeInit(sht4x.hi2c) != HAL_OK)
		{
			Error_Handler();
		}

		HAL_Delay(0);

		HAL_I2C_MspDeInit(sht4x.hi2c);

		HAL_Delay(0);

		HAL_I2C_MspInit(sht4x.hi2c);

		HAL_Delay(0);

		if (HAL_I2C_Init(sht4x.hi2c) != HAL_OK)
		{
			Error_Handler();
		}

		HAL_Delay(0);

		sht4x_Init(sht4x.hi2c);

		SEGGER_RTT_printf(0,"Error Check Process OK !\r\n");
	}
}
void softReset()
{
	i2c_read(SHT4x_SOFTRESET);
}
uint8_t crc8(const uint8_t *data, int len)
{
	const uint8_t POLYNOMIAL = 0x31;
	uint8_t crc = 0xFF;

	for (int j = len; j; --j) {
		crc ^= *data++;

		for (int i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
		}
	}
	return crc;
}
