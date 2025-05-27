/*
 * TMP112.h
 *
 *  Created on: May 21, 2025
 *      Author: Step
 */

#ifndef INC_TMP112_H_
#define INC_TMP112_H_

#include "main.h"

#define TMP112_ADDR           (0x48 << 1)  // ADD0 pini GND ise
#define TMP112_TEMP_REG       0x00
#define TMP112_CONFIG_REG     0x01

typedef struct {
    I2C_HandleTypeDef *hi2c;
} TMP112;

typedef enum {
    TMP112_RESOLUTION_9_BIT  = 0x00,
    TMP112_RESOLUTION_10_BIT = 0x20,
    TMP112_RESOLUTION_11_BIT = 0x40,
    TMP112_RESOLUTION_12_BIT = 0x60
} TMP112_Resolution;

HAL_StatusTypeDef TMP112_Init(TMP112 *sensor, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef TMP112_ReadTemperature(TMP112 *sensor, float *temperature);


HAL_StatusTypeDef TMP112_SetResolution(TMP112 *sensor, TMP112_Resolution resolution);
HAL_StatusTypeDef TMP112_EnableShutdownMode(TMP112 *sensor, uint8_t enable);
HAL_StatusTypeDef TMP112_TriggerOneShot(TMP112 *sensor);

#endif /* INC_TMP112_H_ */
