/*
 * hdc1080.h
 *
 *  Created on: Apr 17, 2025
 *      Author: Step
 */


#include "main.h"


#define HDC1080_ADDR            (0x40 << 1)  // I2C adresi (7-bit left shift)
#define HDC1080_TEMP_REG        0x00
#define HDC1080_HUMIDITY_REG    0x01
#define HDC1080_CONFIG_REG      0x02

#define HDC1080_RESOLUTION_TEMP_14BIT     0x00
#define HDC1080_RESOLUTION_TEMP_11BIT     0x04

#define HDC1080_RESOLUTION_HUM_14BIT      0x00
#define HDC1080_RESOLUTION_HUM_11BIT      0x01
#define HDC1080_RESOLUTION_HUM_8BIT       0x02

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t serialNumber[3];
} HDC1080;


HAL_StatusTypeDef HDC1080_Init(HDC1080 *sensor, I2C_HandleTypeDef *hi2c, uint8_t tempRes, uint8_t humRes);
HAL_StatusTypeDef HDC1080_ReadTemperatureAndHumidity(HDC1080 *sensor, float *temperature, float *humidity);
