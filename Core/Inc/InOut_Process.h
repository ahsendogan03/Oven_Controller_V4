/*
 * InOut_Process.h
 *
 *  Created on: Jan 21, 2025
 *      Author: Step
 */

#ifndef INC_INOUT_PROCESS_H_
#define INC_INOUT_PROCESS_H_

#include "main.h"

#define HIGH 	1
#define LOW		0

#define K17 	0x8
#define K16 	0x10
#define K15 	0x20
#define K14 	0x40
#define BUZZER 	0x80
#define K8 		0x200
#define K9 		0x400
#define K10 	0x800
#define K11 	0x1000
#define K12 	0x2000
#define K13 	0x4000
#define K18 	0x8000
#define K1 		0x20000//
#define K2 		0x40000
#define K3 		0x80000//
#define K4 		0x100000
#define K5 		0x200000//
#define K6 		0x400000//
#define K7 		0x800000

#define Q_MAYALAMA_KAZAN_ISITICISI 	K1
#define Q_MAYALAMA_ISITICI			K2
#define Q_MAYALAMA_FAN				K3
#define Q_MAYALAMA_LAMBA			K4
#define Q_MAYALAMA_SAMANDIRA		K5
#define Q_KORNA						K6
#define Q_KLAPE						K7

#define Q_ISITICI_YADA_BRULOR		K8
#define Q_FAN_SOL_YAVAS				K9
#define Q_FAN_SAG_YAVAS				K10
#define Q_ASPIRATOR					K11
#define Q_LAMBA						K12
#define Q_BUHAR_VALFI				K13
#define Q_PANO_SOGUTUCU_FAN			K14
#define Q_TEPSI_SOL					K15
#define Q_TEPSI_SAG					K16
#define Q_FAN_SOL_HIZLI				K16
#define Q_FAN_SAG_HIZLI				K17


#define I_KAPI_SWITCH			GPIOB, GPIO_PIN_10
#define I_BUHAR_HAZIR			GPIOC, GPIO_PIN_6


#define LATCH_PIN GPIO_PIN_2
#define LATCH_PORT GPIOD

#define OE_PIN	GPIO_PIN_7
#define OE_PORT GPIOA

#define RUN_LED GPIOC, GPIO_PIN_13


void setOut(uint32_t outputAddr, uint8_t status);
void resetBit(uint32_t *data, uint32_t bitMask);
void setBit(uint32_t *data, uint32_t bitMask);
void ShiftRegister_SendData(uint32_t data);
void setOutData(uint32_t outputAddr, uint8_t status);
void shiftRefresh(void);


#endif /* INC_INOUT_PROCESS_H_ */
