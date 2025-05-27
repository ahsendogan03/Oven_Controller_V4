/*
 * InOut_Process.c
 *
 *  Created on: Jan 21, 2025
 *      Author: Step
 */


#include "InOut_Process.h"
#include "spi.h"
#include "SEGGER_RTT.h"

uint32_t outputData = 0;

void ShiftRegister_SendData(uint32_t data)
{
    // Latch pinini LOW yap
    HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_RESET);

    // Veriyi parçalayarak 4 byte halinde gönder
    uint8_t bytes[4];
    bytes[0] = (data >> 24) & 0xFF;  // En yüksek bayt
    bytes[1] = (data >> 16) & 0xFF;
    bytes[2] = (data >> 8) & 0xFF;
    bytes[3] = data & 0xFF;         // En düşük bayt

    // 4 byte'ı sırasıyla gönder
    HAL_SPI_Transmit(&hspi1, bytes, 4, HAL_MAX_DELAY);

    // Latch pinini HIGH yap
    HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_SET);
}

void setBit(uint32_t *data, uint32_t bitMask)
{
    *data |= bitMask;
}

// Belirtilen bit maskesine göre biti reset eder (0 yapar)
void resetBit(uint32_t *data, uint32_t bitMask)
{
    *data &= ~bitMask;
}

void setOut(uint32_t outputAddr, uint8_t status)
{
	(status == 1) ? setBit(&outputData,outputAddr) : resetBit(&outputData,outputAddr);
	ShiftRegister_SendData(outputData);
}

void setOutData(uint32_t outputAddr, uint8_t status)
{
	(status == 1) ? setBit(&outputData,outputAddr) : resetBit(&outputData,outputAddr);
}

void shiftRefresh(void)
{
	ShiftRegister_SendData(outputData);
}
