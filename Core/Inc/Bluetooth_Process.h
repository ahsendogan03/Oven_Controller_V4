/*
 * Bluetooth_Process.h
 *
 *  Created on: Feb 4, 2025
 *      Author: Step
 */

#ifndef INC_BLUETOOTH_PROCESS_H_
#define INC_BLUETOOTH_PROCESS_H_

#include "main.h"

// Modbus komut kodları
#define MB_READ_CMD           	0x03
#define MB_WRITE_CMD          	0x10
#define TARGET_READ_REGISTER  	0x99

#define MAX_WRITE_REGISTERS   100

#define BLUETOOTH_TIMEOUT_MS	4000
#define DEBUG_ESP32 1

#define BL_MAGIC_NUMBER   0xB007
#define BL_MAGIC_REG      BKP->DR1

typedef enum
{
	ESP32_NO_RESPONSE	= 	0,
	ESP32_CRC_ERROR		= 	1,
	ESP32_WRITE_OK		=	2,
	ESP32_READ_OK		=	3,

} ESP32_Response;


HAL_StatusTypeDef ESP32_SetUsartChannel(UART_HandleTypeDef *huart, USART_TypeDef *Declaration, DMA_HandleTypeDef *hdma);
void ESP32_receiveDataProcess(void);
void Bluetooth_run(void);
void Bluetooth_dwinWrite(uint16_t addr, uint16_t value);
void Jump_To_Bootloader(void);
ESP32_Response ESP32_writeRegister(uint16_t* pBuffer, uint16_t addr, uint8_t len);
ESP32_Response ESP32_receiveDataCheck(void);
void STM32_RequestCheck_Process(void);
void STM32_RequestBufferWrite(uint16_t* pBuffer, uint16_t addr, uint8_t len);
void STM32_RequestReadyCounter(void);

#endif /* INC_BLUETOOTH_PROCESS_H_ */
