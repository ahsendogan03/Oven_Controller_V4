/*
 * Flash_Process.c
 *
 *  Created on: Apr 17, 2026
 *      Author: Step
 */


#include "Flash_Process.h"


HAL_StatusTypeDef Flash_IsAddressValid(uint32_t address, uint32_t length)
{
    if ((address % 4) != 0)
        return HAL_ERROR;

    uint32_t endAddr = address + (length * 4);

    if (address < FLASH_BASE_ADDR || endAddr > FLASH_END_ADDR)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef Flash_Erase_Sector(uint32_t sectorNumber)
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;

    uint32_t sectorAddress = FLASH_SECTOR_ADDR(sectorNumber);

    HAL_FLASH_Unlock();

    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = sectorAddress;
    eraseInit.NbPages     = 1;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef Flash_Read(uint32_t address,
                             uint8_t *data,
                             uint32_t length)
{
    /* Parametre kontrolleri */
    if (data == NULL || length == 0)
        return HAL_ERROR;

    /* Flash adres aralığı kontrolü */
    if (Flash_IsAddressValid(address, length) != HAL_OK)
        return HAL_ERROR;

    /* Okuma */
    for (uint32_t i = 0; i < length; i++)
    {
        data[i] = *(volatile uint8_t *)(address + i);
    }

    return HAL_OK;
}



HAL_StatusTypeDef Flash_Write(uint32_t address,
                              uint32_t *data,
                              uint32_t length)
{
    HAL_StatusTypeDef status;

    if (data == NULL)
        return HAL_ERROR;

    if (Flash_IsAddressValid(address, length) != HAL_OK)
        return HAL_ERROR;

    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < length; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   address + (i * 4),
                                   data[i]);

        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    /* ===== VERIFY ===== */
    for (uint32_t i = 0; i < length; i++)
    {
        uint32_t flashData =
            *(volatile uint32_t *)(address + (i * 4));

        if (flashData != data[i])
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}
