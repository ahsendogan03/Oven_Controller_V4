/*
 * EEPROM_Process.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Step
 */

#ifndef INC_EEPROM_PROCESS_H_
#define INC_EEPROM_PROCESS_H_

#include "main.h"

#define EEPROM_BASE_ADDR  			0x50  // 7-bit temel adres
#define EEPROM_PAGE_SIZE         	64    // EEPROM sayfa boyutu
#define EEPROM_TIMEOUT_MS 			20
#define EEPROM_USAGE_CHECK_ADDR 	0
#define EEPROM_USAGE_CHECK_VAL		99

#define EEPROM_PAGE_LIMIT 65536   // 64 kB EEPROM
#define EEPROM_MAX_READ_CHUNK 128 // I2C donanımı sınırı için güvenli parça boyutu


typedef enum
{
    EE_WRITE_ERROR     	=   1,
    EE_WRITE_OK        	=   2,
    EEPROM_BUSY     	=   3,

} EEPROM_writeResponse;

typedef enum
{
    EE_INIT_OK         =   1,
    EE_INIT_ERROR      =   2,

} EEPROM_initResponse;

EEPROM_writeResponse AT24C256_WaitForReady(I2C_HandleTypeDef *hi2c);
EEPROM_writeResponse EEPROM_Write(I2C_HandleTypeDef *hi2c, uint16_t mem_address, uint8_t *data, uint16_t len);
void EEPROM_Read(I2C_HandleTypeDef *hi2c, uint16_t mem_address, uint8_t *data, uint16_t len);
HAL_StatusTypeDef AT24C256_Write(I2C_HandleTypeDef *hi2c,uint16_t memAddress, uint8_t *data, uint16_t dataSize);
HAL_StatusTypeDef EEPROM_Read_Safe(I2C_HandleTypeDef *hi2c, uint16_t mem_address, uint8_t *data, uint16_t total_len);
EEPROM_initResponse EEPROM_Recete_DefaultWrite(I2C_HandleTypeDef *hi2c);
void EEPROM_Recete_Read(I2C_HandleTypeDef *hi2c);
EEPROM_initResponse EEPROM_init(I2C_HandleTypeDef *hi2c);
EEPROM_initResponse EEPROM_OtomatikAcma_DefaultWrite(I2C_HandleTypeDef *hi2c);
void EEPROM_OtomatikAcma_Read(I2C_HandleTypeDef *hi2c);

#define EE_RECETE_ILK_ADR 			0x2000
#define EE_RECETE_DATA_SIZE			60
#define EE_RECETE_SON_ADR			EE_RECETE_ILK_ADR + ((EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)*DW_RECETE_AMOUNT)

#define EE_OTOMATIK_ACMA_ILK_ADR 	EE_RECETE_SON_ADR
#define EE_OTOMATIK_ACMA_PARAM_SIZE	14
#define EE_OTOMATIK_PISIRME_INFO_ADR	EE_OTOMATIK_ACMA_ILK_ADR + 10
#define EE_OTOMATIK_AKTIF_INFO_ADR		EE_OTOMATIK_ACMA_ILK_ADR + 12


//#define EE_RECETE_RESIM_ILK_ADR 	0x768
//#define EE_RECEDE_ADIM_SAYISI_ADR	0x2000

#endif /* INC_EEPROM_PROCESS_H_ */
