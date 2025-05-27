/*
 * TMP112.c
 *
 *  Created on: May 21, 2025
 *      Author: Step
 */


#include "TMP112.h"

HAL_StatusTypeDef TMP112_Init(TMP112 *sensor, I2C_HandleTypeDef *hi2c)
{
    sensor->hi2c = hi2c;

    // Sensör hazır mı kontrol et
    if (HAL_I2C_IsDeviceReady(sensor->hi2c, TMP112_ADDR, 3, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Konfigürasyon (varsayılan ayarları kullanabiliriz)
    // İstenirse çözünürlük, shutdown mode, vs. burada yapılandırılabilir

    return HAL_OK;
}

HAL_StatusTypeDef TMP112_ReadTemperature(TMP112 *sensor, float *temperature)
{
    uint8_t reg = TMP112_TEMP_REG;
    uint8_t data[2];
    float offset = 1.0;

    if (HAL_I2C_Master_Transmit(sensor->hi2c, TMP112_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(sensor->hi2c, TMP112_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    int16_t rawTemp = (data[0] << 4) | (data[1] >> 4);

    // Negatif sıcaklıklar için işaret düzeltmesi
    if (rawTemp & 0x800)
        rawTemp |= 0xF000;

    *temperature = (rawTemp * 0.0625f) - offset;  // Her adım 0.0625 °C

    return HAL_OK;
}

HAL_StatusTypeDef TMP112_SetResolution(TMP112 *sensor, TMP112_Resolution resolution)
{
    uint8_t config[3];
    config[0] = TMP112_CONFIG_REG;

    if (HAL_I2C_Master_Transmit(sensor->hi2c, TMP112_ADDR, &config[0], 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(sensor->hi2c, TMP112_ADDR, &config[1], 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    config[2] &= ~(0x60);  // Resolution bitlerini temizle
    config[2] |= resolution;

    config[0] = TMP112_CONFIG_REG;
    if (HAL_I2C_Mem_Write(sensor->hi2c, TMP112_ADDR, config[0], I2C_MEMADD_SIZE_8BIT, &config[1], 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef TMP112_EnableShutdownMode(TMP112 *sensor, uint8_t enable)
{
    uint8_t config[3];
    config[0] = TMP112_CONFIG_REG;

    if (HAL_I2C_Master_Transmit(sensor->hi2c, TMP112_ADDR, &config[0], 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(sensor->hi2c, TMP112_ADDR, &config[1], 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (enable)
        config[1] |= 0x01;  // Shutdown bit ON
    else
        config[1] &= ~0x01; // Shutdown bit OFF

    if (HAL_I2C_Mem_Write(sensor->hi2c, TMP112_ADDR, TMP112_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config[1], 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef TMP112_TriggerOneShot(TMP112 *sensor)
{
    uint8_t config[3];
    config[0] = TMP112_CONFIG_REG;

    if (HAL_I2C_Master_Transmit(sensor->hi2c, TMP112_ADDR, &config[0], 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(sensor->hi2c, TMP112_ADDR, &config[1], 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    config[1] |= 0x80;  // OS (One-Shot) bit set

    if (HAL_I2C_Mem_Write(sensor->hi2c, TMP112_ADDR, TMP112_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config[1], 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Ölçümün tamamlanması için birkaç milisaniye beklenebilir
    HAL_Delay(30);  // Worst case 27ms (datasheet’e göre)

    return HAL_OK;
}
