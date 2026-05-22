/*
 * Flash_Process.h
 *
 *  Created on: Apr 17, 2026
 *      Author: Step
 */

#ifndef INC_FLASH_PROCESS_H_
#define INC_FLASH_PROCESS_H_

#include "main.h"

#define FLASH_PACKET_SIZE     	128 // byte


#define FLASH_BASE_ADDR     	FLASH_BASE
#define FLASH_END_ADDR			FLASH_BANK1_END

#define FLASH_SECTOR_SIZE   	0x800U      // 2 KB

#define SLOT_DATA_SIZE			110			// KB (max

#define SLOT1_START_ADDR		0x08005000U
#define SLOT1_START_SECTOR		10
#define SLOT1_END_SECTOR		SLOT1_START_SECTOR + (SLOT_DATA_SIZE/2) - 1


#if ((((SLOT_DATA_SIZE * 2)/2)*FLASH_SECTOR_SIZE) + 0x08005000U) < FLASH_END_ADDR
#define SLOT2_START_ADDR	SLOT1_START_ADDR + ((SLOT_DATA_SIZE/2)*FLASH_SECTOR_SIZE)
#define SLOT2_START_SECTOR		SLOT1_END_SECTOR + 1
#define SLOT2_END_SECTOR		SLOT2_START_SECTOR + (SLOT_DATA_SIZE/2) - 1
#endif

#define FLASH_INFO_ADDR	SLOT1_START_ADDR - FLASH_SECTOR_SIZE
#define FLASH_INFO_SECTOR (FLASH_INFO_ADDR - FLASH_BASE_ADDR) / FLASH_SECTOR_SIZE

#define FLASH_INFO_CHECK_NUMBER	0x99

#define FLASH_SECTOR_ADDR(sector) (FLASH_BASE_ADDR + ((sector) * FLASH_SECTOR_SIZE))

#define SAFE_PROGRAM_DECL_WAIT	60000


HAL_StatusTypeDef Flash_IsAddressValid(uint32_t address, uint32_t length);
HAL_StatusTypeDef Flash_Erase_Sector(uint32_t sectorNumber);
HAL_StatusTypeDef Flash_Read(uint32_t address,
                             uint8_t *data,
                             uint32_t length);
HAL_StatusTypeDef Flash_Write(uint32_t address,
                              uint32_t *data,
                              uint32_t length);

#endif /* INC_FLASH_PROCESS_H_ */
