/*
 * sht40.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Step
 */

#ifndef INC_SHT40_H_
#define INC_SHT40_H_


#include "main.h"

#define SHT4x_DEFAULT_ADDR 0x44 /**< SHT4x I2C Address */


#define SHT4x_READSERIAL 0x89 /**< Read Out of Serial Register */
#define SHT4x_SOFTRESET 0x94  /**< Soft Reset */
#define Init_OK 1
#define CRC_ERR 10

typedef enum
{
	SHT4x_NOHEAT_HIGHPRECISION	= 	0xFD,
	SHT4x_NOHEAT_MEDPRECISION	= 	0xF6,
	SHT4x_NOHEAT_LOWPRECISION	= 	0xE0,

} sht4x_mode_t;

typedef enum
{
	SHT4x_HIGHHEAT_1S		= 	0x39,
	SHT4x_HIGHHEAT_100MS	= 	0x32,

	SHT4x_MEDHEAT_1S		= 	0x2F,
	SHT4x_MEDHEAT_100MS		=	0x24,

	SHT4x_LOWHEAT_1S		=	0x1E,
	SHT4x_LOWHEAT_100MS		=	0x15,

} sht4x_heat_t;

uint8_t sht4x_Init();
void sht4x_setMode(uint8_t cmd);
uint8_t crc8(const uint8_t *data, int len);
uint8_t sht4xGetEvent(float *pTemp, float *pHum);
uint8_t sht4xGetHeat(sht4x_heat_t cmd, float *pTemp, float *pHum);
void errorCheckProcess();
void softReset();


#endif /* INC_SHT40_H_ */
