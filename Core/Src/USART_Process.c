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





//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
////	if(huart->Instance == DWIN_usartDeclaration)
////	{
////		DWIN_receiveDataProcess(Size);
////
////		HAL_UARTEx_ReceiveToIdle_DMA(DWIN_huart_channel, DWIN_rxBuffer, DWIN_rxBufferSize);
////		__HAL_DMA_DISABLE_IT(DWIN_hdma_usart_purpose, DMA_IT_HT);
////
////	}
//	if(huart->Instance == ESP32_usartDeclaration)
//	{
//		ESP32_receiveDataProcess(Size);
//		HAL_UARTEx_ReceiveToIdle_DMA(ESP32_huart_channel, ESP32_rxBuffer, ESP32_rxBufferSize);
//		__HAL_DMA_DISABLE_IT(ESP32_hdma_usart_purpose, DMA_IT_HT);
//
//	}
//}
