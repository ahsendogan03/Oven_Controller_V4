/*
 * USART_Process.c
 *
 *  Created on: Jan 20, 2025
 *      Author: Step
 */


#include "USART_Process.h"
#include "DWIN_Process.h"
#include "InOut_Process.h"
#include "Bluetooth_Process.h"

uint8_t DWIN_rxBuffer[DWIN_rxBufferSize];
uint8_t main_DWIN_rxBuffer[DWIN_rxBufferSize];


uint8_t ESP32_rxBuffer[ESP32_RX_BUFFER_SIZE];
uint8_t main_ESP32_rxBuffer[ESP32_RX_BUFFER_SIZE];

USART_TypeDef *DWIN_usartDeclaration;
UART_HandleTypeDef *DWIN_huart_channel;
DMA_HandleTypeDef *DWIN_hdma_usart_purpose;

USART_TypeDef *ESP32_usartDeclaration;
UART_HandleTypeDef *ESP32_huart_channel;
DMA_HandleTypeDef *ESP32_hdma_usart_purpose;


usartInfo DWIN;
usartInfo ESP32;
