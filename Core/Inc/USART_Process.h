/*
 * USART_Process.h
 *
 *  Created on: Jan 20, 2025
 *      Author: Step
 */

#ifndef INC_USART_PROCESS_H_
#define INC_USART_PROCESS_H_

#include "main.h"
#include "usart.h"


#define DWIN_rxBufferSize 	50
#define ESP32_rxBufferSize 	50

typedef struct{
	uint8_t rxDoneFlag;
    uint8_t Init;
}usartInfo;


#endif /* INC_USART_PROCESS_H_ */
