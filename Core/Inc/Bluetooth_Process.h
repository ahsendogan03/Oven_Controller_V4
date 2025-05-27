/*
 * Bluetooth_Process.h
 *
 *  Created on: Feb 4, 2025
 *      Author: Step
 */

#ifndef INC_BLUETOOTH_PROCESS_H_
#define INC_BLUETOOTH_PROCESS_H_

#include "main.h"

#define MB_READ_CMD 			0x03
#define MB_WRITE_CMD			0x16
#define BLUETOOTH_TIMEOUT_MS	4000

HAL_StatusTypeDef ESP32_SetUsartChannel(UART_HandleTypeDef *huart, USART_TypeDef *Declaration, DMA_HandleTypeDef *hdma);
void ESP32_receiveDataProcess(void);
void Bluetooth_run(void);
void Bluetooth_dwinWrite(uint16_t addr, uint16_t value);

#endif /* INC_BLUETOOTH_PROCESS_H_ */
