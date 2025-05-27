/*
 * hdc1080.c
 *
 *  Created on: Apr 17, 2025
 *      Author: Step
 */


#include "hdc1080.h"

static HAL_StatusTypeDef HDC1080_ReadRegister16(HDC1080 *sensor, uint8_t reg, uint16_t *value)
{
    if (HAL_I2C_Master_Transmit(sensor->hi2c, HDC1080_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    uint8_t data[2];
    if (HAL_I2C_Master_Receive(sensor->hi2c, HDC1080_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    *value = (data[0] << 8) | data[1];
    return HAL_OK;
}

HAL_StatusTypeDef HDC1080_Init(HDC1080 *sensor, I2C_HandleTypeDef *hi2c, uint8_t tempRes, uint8_t humRes)
{
    sensor->hi2c = hi2c;

    if (HAL_I2C_IsDeviceReady(sensor->hi2c, HDC1080_ADDR, 3, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Seri numara
    if (HDC1080_ReadRegister16(sensor, 0xFB, &sensor->serialNumber[0]) != HAL_OK) return HAL_ERROR;
    if (HDC1080_ReadRegister16(sensor, 0xFC, &sensor->serialNumber[1]) != HAL_OK) return HAL_ERROR;
    if (HDC1080_ReadRegister16(sensor, 0xFD, &sensor->serialNumber[2]) != HAL_OK) return HAL_ERROR;

    // Config byte'larını oluştur (bit: 10-11: temp, 0-1: hum)
    uint8_t config_msb = (tempRes & 0x04);  // sadece bit 2
    uint8_t config_lsb = (humRes & 0x03);   // sadece bit 0-1

    uint8_t configData[3] = {
        HDC1080_CONFIG_REG,
        config_msb,
        config_lsb
    };

    if (HAL_I2C_Master_Transmit(sensor->hi2c, HDC1080_ADDR, configData, 3, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef HDC1080_ReadTemperatureAndHumidity(HDC1080 *sensor, float *temperature, float *humidity)
{
    uint8_t reg;

    // 🔹 Sıcaklık ölçümü başlat
    reg = HDC1080_TEMP_REG;
    if (HAL_I2C_Master_Transmit(sensor->hi2c, HDC1080_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(15);  // Sıcaklık ölçüm süresi (datasheet’e göre 6.35ms @14-bit)

    uint8_t tempData[2];
    if (HAL_I2C_Master_Receive(sensor->hi2c, HDC1080_ADDR, tempData, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    uint16_t tempRaw = (tempData[0] << 8) | tempData[1];
    *temperature = ((float)tempRaw / 65536.0f) * 165.0f - 40.0f;

    // 🔹 Nem ölçümü başlat
    reg = HDC1080_HUMIDITY_REG;
    if (HAL_I2C_Master_Transmit(sensor->hi2c, HDC1080_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(15);  // Nem ölçüm süresi (datasheet’e göre 6.50ms @14-bit)

    uint8_t humData[2];
    if (HAL_I2C_Master_Receive(sensor->hi2c, HDC1080_ADDR, humData, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    uint16_t humRaw = (humData[0] << 8) | humData[1];
    *humidity = ((float)humRaw / 65536.0f) * 100.0f;

    return HAL_OK;
}
