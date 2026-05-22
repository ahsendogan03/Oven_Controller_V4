/*
 * Bluetooth_Process.c
 *
 *  Created on: Feb 4, 2025
 *      Author: Step
 */

#include "Bluetooth_Process.h"
#include "USART_Process.h"
#include "DWIN_Process.h"
#include "SEGGER_RTT.h"
#include "string.h"
#include "InOut_Process.h"
#include "EEPROM_Process.h"
#include "DWIN_Adress.h"
#include "stdio.h"
#include "PID_Control.h"
#include "rtc.h"

#define MEM_READ_32(addr) (*(volatile uint32_t *)(addr))

extern USART_TypeDef *ESP32_usartDeclaration;
extern UART_HandleTypeDef *ESP32_huart_channel;
extern DMA_HandleTypeDef *ESP32_hdma_usart_purpose;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern usartInfo ESP32;

extern I2C_HandleTypeDef hi2c1;

extern tickCounter counterTick;

extern uint8_t ESP32_rxBuffer[ESP32_RX_BUFFER_SIZE];
extern uint8_t main_ESP32_rxBuffer[ESP32_RX_BUFFER_SIZE];
extern uint16_t registerTable[REGISTER_TABLE_SIZE];

extern uint16_t pisirmeManuelDownCounter;
extern uint16_t buharManuelDownCounter;
extern uint8_t pisirmeSonuAlarmFlag;
extern uint8_t pisirmeSonuAlarmBuzzer;

extern uint8_t ustOnTurbo 		;
extern uint8_t ustArkaTurbo 	;
extern uint8_t altTurbo			;

uint16_t ESP32_writeData[100];
uint16_t ESP32_writeAmountAddress = 0;
uint16_t ESP32_writeAddress = 0;

uint8_t esp32_txBuffer[255];

uint8_t stm32RequestReadyCheck = 0;
uint16_t stm32RequestReadyCheckCounter = 0;
uint8_t stm32RequestFlag = 0;

uint16_t stm32RequestData[100] 	= {0};
uint16_t stm32RequestAddr 		= 0;
uint8_t stm32RequestLength 		= 0;
uint32_t stm32RequestPeriodTick = 0;

uint8_t otomatikAcmaWriteCheck = 0;
uint16_t receteDuzenlemeAdr = 0;

void hexToString(const uint8_t *input, uint16_t inputLen, uint8_t *output)
{
    uint16_t i;

    for(i = 0; i < inputLen; i++)
    {
        // Her byte'ı 2 karakterlik HEX stringe çevir
        sprintf((char *)&output[i * 2], "%02X", input[i]);
    }

    // String sonlandırıcı
    output[inputLen * 2] = '\0';
}

HAL_StatusTypeDef ESP32_SetUsartChannel(UART_HandleTypeDef *huart, USART_TypeDef *Declaration, DMA_HandleTypeDef *hdma)
{
	HAL_StatusTypeDef response;

	ESP32_usartDeclaration 		= Declaration;
	ESP32_huart_channel 		= huart;
	ESP32_hdma_usart_purpose 	= hdma;

	response = HAL_UART_Receive_DMA(huart, main_ESP32_rxBuffer, ESP32_RX_BUFFER_SIZE);

	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	HAL_Delay(0);

	ESP32.rxDoneFlag = 0;

	return response;
}

void ESP32_receiveDataProcess(void)
{
    // Minimum 4 byte kontrolü (Slave + Func + CRC)
    if(ESP32_rxBuffer[0] == 0)
    {
        #if DEBUG_ESP32 == 1
        SEGGER_RTT_printf(0, "ESP32: Empty frame received\r\n");
        #endif
        return;
    }

    // Frame boyutunu belirle
    uint8_t size = 0;

    switch(ESP32_rxBuffer[1])
    {
        case MB_READ_CMD:
            size = 8;  // [Slave][Func][AddrH][AddrL][QtyH][QtyL][CRC_L][CRC_H]
            break;

        case TARGET_READ_REGISTER:
        {
            uint8_t numOfAddresses = ESP32_rxBuffer[2];
            size = 3 + (numOfAddresses * 2) + 2;  // [Slave][Func][Count][Addrs...][CRC]
            break;
        }

        case MB_WRITE_CMD:
            if(ESP32_rxBuffer[6] > 0)
                size = ESP32_rxBuffer[6] + 9;  // [Slave][Func][Addr][Qty][BC][Data...][CRC]
            else
                size = 8;  // Minimum write frame
            break;

        default:
            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32: Unknown command: 0x%02X\r\n", ESP32_rxBuffer[1]);
            #endif
            return;
    }

    // Boyut kontrolü
    if(size < 4 || size > ESP32_RX_BUFFER_SIZE)
    {
        #if DEBUG_ESP32 == 1
        SEGGER_RTT_printf(0, "ESP32: Invalid frame size: %d\r\n", size);
        #endif
        return;
    }

    // CRC kontrolü
    uint16_t crc = calculateCRC16Modbus(ESP32_rxBuffer, size - 2);
    uint16_t crcRx = combineBytes(ESP32_rxBuffer[size - 1], ESP32_rxBuffer[size - 2]);

    if((crc != crcRx) || (crcRx == 0))
    {
        #if DEBUG_ESP32 == 1
        SEGGER_RTT_printf(0, "ESP32 CRC ERROR! Calculated: 0x%04X, Received: 0x%04X\r\n", crc, crcRx);
        #endif
        return;
    }

    #if DEBUG_ESP32 == 1
    SEGGER_RTT_printf(0, "ESP32 RX [%d bytes]: ", size);
    for(int i = 0; i < size && i < 20; i++)
    {
        SEGGER_RTT_printf(0, "%02X ", ESP32_rxBuffer[i]);
    }
    if(size > 20) SEGGER_RTT_printf(0, "...");
    SEGGER_RTT_printf(0, "\r\n");
    #endif

    // Komutları işle
    switch (ESP32_rxBuffer[1])
    {
        // ====================================================================
        // MODBUS READ COMMAND (0x03)
        // ====================================================================
        case MB_READ_CMD:
        {
            uint16_t addr = combineBytes(ESP32_rxBuffer[2], ESP32_rxBuffer[3]);
            uint16_t numOfRegister = combineBytes(ESP32_rxBuffer[4], ESP32_rxBuffer[5]);

            // Güvenlik kontrolü
            if(numOfRegister == 0 || numOfRegister > 125)
            {
                #if DEBUG_ESP32 == 1
                SEGGER_RTT_printf(0, "ESP32 Read: Invalid register count: %d\r\n", numOfRegister);
                #endif
                break;
            }

            // Address sınır kontrolü
            if(addr + numOfRegister > REGISTER_TABLE_SIZE)
            {
                #if DEBUG_ESP32 == 1
                SEGGER_RTT_printf(0, "ESP32 Read: Address out of bounds\r\n");
                #endif
                break;
            }

            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32 Read: %d registers from addr 0x%04X\r\n", numOfRegister, addr);
            #endif

            // Yanıt frame'i hazırla
            uint8_t txBufferSize = (numOfRegister * 2) + 5;

            esp32_txBuffer[0] = 0x01;
            esp32_txBuffer[1] = MB_READ_CMD;
            esp32_txBuffer[2] = (uint8_t)(numOfRegister * 2);

            uint8_t highByte, lowByte;
            for(int i = 0; i < numOfRegister; i++)
            {
                parse16BitTo8Bit(registerTable[addr + i], &highByte, &lowByte);
                esp32_txBuffer[3 + (i * 2)] = highByte;
                esp32_txBuffer[4 + (i * 2)] = lowByte;
            }

            // CRC ekle
            uint16_t crc = calculateCRC16Modbus(esp32_txBuffer, txBufferSize - 2);
            parse16BitTo8Bit(crc, &highByte, &lowByte);

            esp32_txBuffer[txBufferSize - 2] = lowByte;
            esp32_txBuffer[txBufferSize - 1] = highByte;

            // Gönder
            HAL_UART_Transmit_IT(ESP32_huart_channel, esp32_txBuffer, txBufferSize);

            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32 Read OK: Sent %d bytes\r\n", txBufferSize);
            #endif

            break;
        }

        // ====================================================================
        // TARGET READ REGISTER COMMAND (0x99)
        // ====================================================================
        case TARGET_READ_REGISTER:
        {
            uint8_t numOfAddresses = ESP32_rxBuffer[2];

            // Güvenlik kontrolü
            if(numOfAddresses == 0 || numOfAddresses > 125)
            {
                #if DEBUG_ESP32 == 1
                SEGGER_RTT_printf(0, "ESP32 Target Read: Invalid address count: %d\r\n", numOfAddresses);
                #endif
                break;
            }

            // Frame boyutu kontrolü
            uint8_t expectedSize = 3 + (numOfAddresses * 2) + 2;
            if(size != expectedSize)
            {
                #if DEBUG_ESP32 == 1
                SEGGER_RTT_printf(0, "ESP32 Target Read: Invalid frame size\r\n");
                #endif
                break;
            }

            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32 Target Read: %d addresses [", numOfAddresses);
            for(int i = 0; i < numOfAddresses && i < 5; i++)
            {
                uint16_t addr = combineBytes(ESP32_rxBuffer[3 + (i * 2)],
                                              ESP32_rxBuffer[4 + (i * 2)]);
                SEGGER_RTT_printf(0, "0x%04X ", addr);
            }
            if(numOfAddresses > 5) SEGGER_RTT_printf(0, "...");
            SEGGER_RTT_printf(0, "]\r\n");
            #endif

            // Yanıt frame'i hazırla
            uint8_t txBufferSize = 3 + (numOfAddresses * 2) + 2;

            esp32_txBuffer[0] = 0x01;
            esp32_txBuffer[1] = TARGET_READ_REGISTER;
            esp32_txBuffer[2] = numOfAddresses;

            // Her adres için değeri oku ve ekle
            uint8_t highByte, lowByte;
            for(int i = 0; i < numOfAddresses; i++)
            {
                uint16_t addr = combineBytes(ESP32_rxBuffer[3 + (i * 2)],
                                              ESP32_rxBuffer[4 + (i * 2)]);

                // Address sınır kontrolü
                if(addr >= REGISTER_TABLE_SIZE)
                {
                    #if DEBUG_ESP32 == 1
                    SEGGER_RTT_printf(0, "ESP32 Target Read: Address 0x%04X out of bounds\r\n", addr);
                    #endif
                    // Hata durumunda 0 gönder
                    esp32_txBuffer[3 + (i * 2)] = 0x00;
                    esp32_txBuffer[4 + (i * 2)] = 0x00;
                }
                else
                {
                    parse16BitTo8Bit(registerTable[addr], &highByte, &lowByte);
                    esp32_txBuffer[3 + (i * 2)] = highByte;
                    esp32_txBuffer[4 + (i * 2)] = lowByte;
                }
            }

            // CRC ekle
            uint16_t crc = calculateCRC16Modbus(esp32_txBuffer, txBufferSize - 2);
            parse16BitTo8Bit(crc, &highByte, &lowByte);

            esp32_txBuffer[txBufferSize - 2] = lowByte;
            esp32_txBuffer[txBufferSize - 1] = highByte;

            // Gönder
            HAL_UART_Transmit_IT(ESP32_huart_channel, esp32_txBuffer, txBufferSize);

            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32 Target Read OK: Sent %d bytes\r\n", txBufferSize);
            #endif

            break;
        }

        // ====================================================================
        // MODBUS WRITE COMMAND (0x10)
        // ====================================================================
        case MB_WRITE_CMD:
        {
            uint16_t addr   = combineBytes(ESP32_rxBuffer[2], ESP32_rxBuffer[3]);
            uint16_t amount = combineBytes(ESP32_rxBuffer[4], ESP32_rxBuffer[5]);
            uint8_t byteCount = ESP32_rxBuffer[6];

            // Güvenlik kontrolü
            if(amount == 0 || amount > MAX_WRITE_REGISTERS)
            {
                #if DEBUG_ESP32 == 1
                SEGGER_RTT_printf(0, "ESP32 Write: Invalid register count: %d\r\n", amount);
                #endif
                break;
            }

            // Byte count kontrolü
            if(byteCount != (amount * 2))
            {
                #if DEBUG_ESP32 == 1
                SEGGER_RTT_printf(0, "ESP32 Write: Invalid byte count: %d\r\n", byteCount);
                #endif
                break;
            }

            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32 Write: %d registers to addr 0x%04X\r\n", amount, addr);
            #endif

            // Write verilerini buffer'a kaydet
            ESP32_writeAddress       = addr;
            ESP32_writeAmountAddress = amount;

            for(int i = 0; i < amount; i++)
            {
                ESP32_writeData[i] = combineBytes(ESP32_rxBuffer[7 + (i * 2)],
                                                   ESP32_rxBuffer[8 + (i * 2)]);

                #if DEBUG_ESP32 == 1
                if(i < 5)  // İlk 5 değeri göster
                {
                    SEGGER_RTT_printf(0, "ESP32_writeData[%d] = 0x%04X\r\n", i, ESP32_writeData[i]);
                }
                #endif
            }

            #if DEBUG_ESP32 == 1
            if(amount > 5)
                SEGGER_RTT_printf(0, "  ... (%d more)\r\n", amount - 5);
            SEGGER_RTT_printf(0, "ESP32 Write queued: Will be processed in main loop\r\n");
            #endif

            // Başarılı yanıt gönder (Modbus Write Response)
            esp32_txBuffer[0] = 0x01;
            esp32_txBuffer[1] = MB_WRITE_CMD;
            esp32_txBuffer[2] = ESP32_rxBuffer[2];  // Address High
            esp32_txBuffer[3] = ESP32_rxBuffer[3];  // Address Low
            esp32_txBuffer[4] = ESP32_rxBuffer[4];  // Quantity High
            esp32_txBuffer[5] = ESP32_rxBuffer[5];  // Quantity Low

            uint16_t crc = calculateCRC16Modbus(esp32_txBuffer, 6);

            uint8_t highByte, lowByte;
            parse16BitTo8Bit(crc, &highByte, &lowByte);

            esp32_txBuffer[6] = lowByte;
            esp32_txBuffer[7] = highByte;

            if(ESP32_writeAddress != STM32_OTA_BEGIN_ADR)
            {
				HAL_UART_Transmit_IT(ESP32_huart_channel, esp32_txBuffer, 8);

				#if DEBUG_ESP32 == 1
				SEGGER_RTT_printf(0, "ESP32 Write ACK sent\r\n");
				#endif
            }

            break;
        }

        default:
        {
            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32: Unknown command 0x%02X\r\n", ESP32_rxBuffer[1]);
            #endif
            break;
        }
    }
}

void Bluetooth_Check(void)
{

	STM32_RequestCheck_Process();

	if((ESP32.rxDoneFlag == 1)&&(stm32RequestFlag == 0))
	{
	    // CRC OK - Bluetooth bağlantı zaman damgası güncelle
	    counterTick.bluetoothCheck = HAL_GetTick();

	    if(stm32RequestReadyCheck == 1)
	    {
	    	stm32RequestReadyCheckCounter = 0;
	    	stm32RequestReadyCheck = 0;
	    	stm32RequestFlag = 0;

	    	for(int i=0;i<stm32RequestLength;i++)
	    		stm32RequestData[i] = 0;

	    }

		ESP32.rxDoneFlag = 0;
		ESP32_receiveDataProcess();
	}

    if ((HAL_GetTick() - counterTick.bluetoothCheck > BLUETOOTH_TIMEOUT_MS) && (registerTable[BLE_DVC_INFO_UPDATE_ADR] == 0))
    {
    	counterTick.bluetoothCheck = HAL_GetTick();

    	HAL_UART_DMAStop(ESP32_huart_channel);

		if (HAL_UART_DeInit(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		HAL_Delay(0);

		if (HAL_UART_Init(&huart1) != HAL_OK)
		{
			Error_Handler();
		}


		if(ESP32_SetUsartChannel(&huart1, USART1, &hdma_usart1_rx) != HAL_OK)
		{
		  SEGGER_RTT_printf(0,"ESP32 Set Usart Channel Error ! \r\n");
		  Error_Handler();
		}

		HAL_Delay(0);
		ESP32.rxDoneFlag = 0;
    }
}


void STM32_RequestReadyCounter(void)
{
	if(stm32RequestReadyCheck == 1)
	{
		stm32RequestReadyCheckCounter++;

		if(stm32RequestReadyCheckCounter > 2000)
		{
			stm32RequestFlag 		= 1;
			stm32RequestReadyCheck 	= 0;
		}
	}
}

void STM32_RequestBufferWrite(uint16_t* pBuffer, uint16_t addr, uint8_t len)
{
	uint8_t numOfRegister 	= len / 2;

	stm32RequestAddr		= addr;
	stm32RequestLength		= len;

	for(int i=0;i<numOfRegister;i++)
		stm32RequestData[i] = pBuffer[i];

	if(registerTable[BLE_DVC_LOCK_ADR] == 1)
	{
		stm32RequestReadyCheck = 1;
		stm32RequestReadyCheckCounter = 0;
	}
}

void STM32_RequestCheck_Process(void)
{
	if(stm32RequestFlag)
	{
		if((HAL_GetTick() - stm32RequestPeriodTick) > 1000)
		{
			stm32RequestPeriodTick = HAL_GetTick();
			ESP32_writeRegister(stm32RequestData, stm32RequestAddr, stm32RequestLength);
			SEGGER_RTT_printf(0,"STM32 Request Send ! Addr: %04X NumOfReg: %d \r\n",stm32RequestAddr,stm32RequestLength/2);
		}
		if(ESP32.rxDoneFlag)
		{
		    // Bluetooth bağlantı zaman damgası güncelle
		    counterTick.bluetoothCheck = HAL_GetTick();

		    if(ESP32_receiveDataCheck() != ESP32_NO_RESPONSE)
		    {
		    	SEGGER_RTT_printf(0,"STM32 Request Receive OK ! \r\n");
		    	stm32RequestFlag = 0;
		    }

			ESP32.rxDoneFlag = 0;

		}
	}
}

ESP32_Response ESP32_receiveDataCheck(void)
{
	ESP32_Response response = ESP32_NO_RESPONSE;

	// Minimum kontrol - buffer boş mu?
	if(ESP32_rxBuffer[0] == 0x00)
	{
		return NO_RESPONSE;
	}

	// Slave ID kontrolü
	if(ESP32_rxBuffer[0] != 0x01)
	{
		#if DEBUG_ESP32 == 1
		SEGGER_RTT_printf(0, "ESP32 INVALID SLAVE ID: 0x%02X\r\n", ESP32_rxBuffer[0]);
		#endif
		return NO_RESPONSE;
	}

	uint8_t functionCode = ESP32_rxBuffer[1];

	// Exception Code kontrolü (MSB set ise exception)
	if(functionCode & 0x80)
	{
		#if DEBUG_ESP32 == 1
		SEGGER_RTT_printf(0, "ESP32 EXCEPTION: Function=0x%02X, Exception Code=0x%02X\r\n",
						  functionCode, ESP32_rxBuffer[2]);
		#endif
		return NO_RESPONSE;
	}

	uint8_t frameLength = 0;

	// Function code'a göre frame uzunluğunu belirle
	if(functionCode == 0x03)  // Read Holding Registers
	{
		uint8_t byteCount = ESP32_rxBuffer[2];
		frameLength = 3 + byteCount + 2;  // Slave ID + FC + Byte Count + Data + CRC
	}
	else if(functionCode == 0x10)  // Write Multiple Registers
	{
		frameLength = 8;  // Slave ID + FC + Start Addr (2) + Quantity (2) + CRC (2)
	}
	else
	{
		#if DEBUG_ESP32 == 1
		SEGGER_RTT_printf(0, "ESP32 UNKNOWN FUNCTION CODE: 0x%02X\r\n", functionCode);
		#endif
		return NO_RESPONSE;
	}

	// CRC kontrolü (Modbus RTU: CRC Little Endian)
	uint16_t receivedCRC = (ESP32_rxBuffer[frameLength - 1] << 8) | ESP32_rxBuffer[frameLength - 2];
	uint16_t calculatedCRC = calculateCRC16Modbus(ESP32_rxBuffer, frameLength - 2);

	if(receivedCRC != calculatedCRC)
	{
		#if DEBUG_ESP32 == 1
		SEGGER_RTT_printf(0, "ESP32 CRC ERROR: Received=0x%04X, Calculated=0x%04X\r\n",
						  receivedCRC, calculatedCRC);
		#endif
		return CRC_ERROR;
	}


	// Function code'a göre yanıt işleme
	if(functionCode == 0x03)  // Read Holding Registers
	{
		#if DEBUG_ESP32 == 1
		uint8_t byteCount = ESP32_rxBuffer[2];
		SEGGER_RTT_printf(0, "ESP32 READ OK: %d bytes received\r\n", byteCount);
		#endif
		response = READ_OK;
	}
	else if(functionCode == 0x10)  // Write Multiple Registers
	{
		#if DEBUG_ESP32 == 1
		uint16_t startAddr = (ESP32_rxBuffer[2] << 8) | ESP32_rxBuffer[3];
		uint16_t quantity = (ESP32_rxBuffer[4] << 8) | ESP32_rxBuffer[5];
		SEGGER_RTT_printf(0, "ESP32 WRITE OK: Addr=0x%04X, Quantity=%d\r\n",
						  startAddr, quantity);
		#endif
		response = WRITE_OK;
	}

	return response;
}

ESP32_Response ESP32_writeRegister(uint16_t* pBuffer, uint16_t addr, uint8_t len)
{
	ESP32_Response response = ESP32_NO_RESPONSE;

	uint8_t numOfRegister = len / 2;

	// Modbus RTU Frame Header
	esp32_txBuffer[0] = 0x01;        // Slave ID
	esp32_txBuffer[1] = 0x10;                 // Function Code: Write Multiple Registers

	// Starting Address (Big Endian)
	esp32_txBuffer[2] = (addr >> 8) & 0xFF;   // Address High Byte
	esp32_txBuffer[3] = addr & 0xFF;          // Address Low Byte

	// Number of Registers (Big Endian)
	esp32_txBuffer[4] = (numOfRegister >> 8) & 0xFF;  // Quantity High Byte
	esp32_txBuffer[5] = numOfRegister & 0xFF;         // Quantity Low Byte

	// Byte Count
	esp32_txBuffer[6] = numOfRegister * 2;

	// Register Data (Big Endian)
	for(int i = 0; i < numOfRegister; i++)
	{
		uint16_t writeData = pBuffer[i];

		esp32_txBuffer[7 + (i*2)] = (writeData >> 8) & 0xFF;  // High Byte
		esp32_txBuffer[8 + (i*2)] = writeData & 0xFF;         // Low Byte
	}

	// Calculate CRC16 for entire frame (except CRC bytes)
	uint16_t crc = calculateCRC16Modbus(esp32_txBuffer, 7 + (numOfRegister*2));

	// CRC bytes (Little Endian for Modbus RTU)
	esp32_txBuffer[7 + (numOfRegister*2)] = crc & 0xFF;           // CRC Low Byte
	esp32_txBuffer[8 + (numOfRegister*2)] = (crc >> 8) & 0xFF;    // CRC High Byte

	memset(ESP32_rxBuffer, 0, sizeof(ESP32_rxBuffer));

	ESP32.rxDoneFlag = 0;

	if(ESP32.rxDoneFlag != 1)
	{
		if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == 0)
		{
			if(huart3.gState == HAL_UART_STATE_READY)
			{
				memset(ESP32_rxBuffer, 0, sizeof(ESP32_rxBuffer));

				ESP32.rxDoneFlag = 0;
				response = HAL_UART_Transmit_IT(ESP32_huart_channel, esp32_txBuffer, 9 + (numOfRegister*2));

			}
		}

	}

	return response;
}

void BLE_otomatikAcmaWriteProcess(void)
{

	uint16_t defaultOtomatik_allData_u16[(EE_OTOMATIK_ACMA_PARAM_SIZE/2) * 7] = {0};
	uint8_t defaultOtomatik_allData_u8[EE_OTOMATIK_ACMA_PARAM_SIZE * 7] = {0};

	for(int i=0;i<((EE_OTOMATIK_ACMA_PARAM_SIZE/2) * 7);i++)
		defaultOtomatik_allData_u16[i] = registerTable[APP_OTOMATIK_ACMA_SAAT_ADR + i];

	convert_u16_to_u8(defaultOtomatik_allData_u16, defaultOtomatik_allData_u8, sizeof(defaultOtomatik_allData_u8));


	if(EEPROM_Write(&hi2c1, EE_OTOMATIK_ACMA_ILK_ADR, defaultOtomatik_allData_u8, sizeof(defaultOtomatik_allData_u8)) != EE_WRITE_OK)
		SEGGER_RTT_printf(0,"BLE_otomatikAcmaWriteProcess -> EEPROM_Write ERR! \r\n");
	else
		EEPROM_OtomatikAcma_Read(&hi2c1);

}

void BLE_receteDuzenlemeProcess(void)
{
	uint8_t receteNum = ((receteDuzenlemeAdr - APP_RECETE_ILK_ADR) + 1) / APP_RECETE_LENGTH;

	uint16_t receteParamData_u16[(EE_RECETE_DATA_SIZE / 2) + (DW_RECETE_ISIM_SIZE / 2)] = {0};
	uint8_t receteParamData_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

	for(int i=0;i<((EE_RECETE_DATA_SIZE / 2) + (DW_RECETE_ISIM_SIZE / 2));i++)
		receteParamData_u16[i] = registerTable[APP_RECETE_ILK_ADR + ((receteNum - 1) * APP_RECETE_LENGTH) + i];

	convert_u16_to_u8(receteParamData_u16, receteParamData_u8, sizeof(receteParamData_u16));

	if(EEPROM_Write(&hi2c1, EE_RECETE_ILK_ADR + ((receteNum - 1) * (EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), receteParamData_u8, sizeof(receteParamData_u8)) != EE_WRITE_OK)
		SEGGER_RTT_printf(0,"BLE_receteDuzenlemeProcess -> EEPROM_Write ERR! \r\n");

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	uint16_t recete_isim_data_u16[DW_RECETE_ISIM_SIZE/2] = {0};

	for(int i=0;i<(DW_RECETE_ISIM_SIZE/2);i++)
		recete_isim_data_u16[i] = registerTable[APP_RECETE_ILK_ADR + ((receteNum - 1) * APP_RECETE_LENGTH) + ((EE_RECETE_DATA_SIZE/2) - 2) + i];

	DWIN_writeRegiser(recete_isim_data_u16, DW_RECETE_ISIM_ILK_ADR + ((receteNum - 1)*(DW_RECETE_ISIM_SIZE)), sizeof(recete_isim_data_u16)); // Her seferinde 10 register atlanıyor

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	uint16_t recete_resim_data_u16 = registerTable[APP_RECETE_ILK_ADR + ((receteNum - 1) * APP_RECETE_LENGTH) + (APP_RECETE_LENGTH - 2)];

	DWIN_writeRegiser(&recete_resim_data_u16, DW_RECETE_RESIM_ILK_ADR + (receteNum - 1), sizeof(recete_resim_data_u16));

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void Bluetooth_writeRegister(void)
{
	if(ESP32_writeAmountAddress > 0)
	{
		for(int i=0;i<ESP32_writeAmountAddress;i++)
		{
			Bluetooth_dwinWrite(ESP32_writeAddress + i, ESP32_writeData[i]);
		}

		ESP32_writeAmountAddress = 0;
		memset(ESP32_writeData,0,sizeof(ESP32_writeData));
	}

	else if(otomatikAcmaWriteCheck > 0)
	{
		BLE_otomatikAcmaWriteProcess();

		otomatikAcmaWriteCheck = 0;
	}

	else if(receteDuzenlemeAdr > 0)
	{
		if((((receteDuzenlemeAdr - APP_RECETE_ILK_ADR) + 1) % APP_RECETE_LENGTH) == 0)
			BLE_receteDuzenlemeProcess();
		else
			SEGGER_RTT_printf(0,"Bluetooth_writeRegister -> ReceteDuzenleme WRONG ADR! \r\n");

		receteDuzenlemeAdr = 0;
	}
}

void Bluetooth_dwinWrite(uint16_t addr, uint16_t value)
{
	uint16_t data = value;
	uint8_t data2[2];

	switch(addr)
	{

		case STM32_OTA_BEGIN_ADR:
			Jump_To_Bootloader();
		break;

		case DW_LAMBA_ADR:

			registerTable[DW_LAMBA_ADR] = data;

			if(data == 0)
			{
				setOut(K12, 0);
				setOut(K13, 0);
			}
			else if(data == 1)
			{
				setOut(K12, 1);
				setOut(K13, 1);

				counterTick.lambaSuresi = HAL_GetTick();
			}

			DWIN_writeRegiser(&data, DW_LAMBA_ADR, sizeof(data));

		break;

		case DW_MANUEL_MOD_GIRIS_ADR:


			if(data == 0)
			{
				registerTable[REG_DW_MODE_INFO_ADR] = DW_ANA_SAYFA_ENTER;

				DWIN_resetManuelPisirme();
				DWIN_changePage(0);
			}

			else if(data == 1)
			{
				registerTable[REG_DW_MODE_INFO_ADR] = DW_MANUEL_MODE_ENTER;

				PID_Setup();
				DWIN_changePage(2);
				setOut(K14, data);
			}


		break;

		case DW_BUHAR_HAZIRLAMA_ADR:

			registerTable[DW_BUHAR_HAZIRLAMA_ADR] = data;

			DWIN_writeRegiser(&data, DW_BUHAR_HAZIRLAMA_ADR, sizeof(data));

			if(data == 0)
			{
				setOut(K8, 0);

				registerTable[DW_BUHAR_HAZIR_ANIM] = data;
				DWIN_writeRegiser(&data, DW_BUHAR_HAZIR_ANIM, sizeof(data));
			}

			else if(data == 1)
			{
				setOut(K8, 1);

				registerTable[DW_BUHAR_HAZIR_ANIM] = data;
				DWIN_writeRegiser(&data, DW_BUHAR_HAZIR_ANIM, sizeof(data));
			}

		break;

		case DW_BUHAR_PUSKURTME_ADR:

			registerTable[DW_BUHAR_PUSKURTME_ADR] = data;

			if(data == 0)
			{
				setOut(K9, 0);

				registerTable[DW_BUHAR_PUSKURTME_ADR] = 0;

				data = registerTable[DW_BUHAR_SURESI_ORT_ADR];
				registerTable[DW_BUHAR_SURESI_ADR] 	= registerTable[DW_BUHAR_SURESI_ORT_ADR];

				DWIN_writeRegiser(&data, DW_BUHAR_SURESI_ADR, sizeof(data));
				DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));

			}

			else if((data == 1)&&(registerTable[DW_BUHAR_HAZIRLAMA_ADR] == 1))
			{
				setOut(K9, 1);

				registerTable[DW_BUHAR_PUSKURTME_ADR] = data;
				buharManuelDownCounter = registerTable[DW_BUHAR_SURESI_ADR] + 1;
				registerTable[DW_BUHAR_SURESI_ORT_ADR] = registerTable[DW_BUHAR_SURESI_ADR];
				DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));
			}

			else if((data == 1)&&(registerTable[DW_BUHAR_HAZIRLAMA_ADR] == 0))
			{
				data = 0;
				registerTable[DW_BUHAR_PUSKURTME_ADR] = data;
				DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));

			}


		break;

		case DW_TURBO_ADR:

			registerTable[DW_TURBO_ADR] = data;

			if(data == 0)
			{
				altTurbo = 0;
				ustOnTurbo = 0;
				ustArkaTurbo = 0;
			}

			DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));

		break;

		case DW_UST_SICAKLIK_SET_ADR:


			registerTable[DW_UST_SICAKLIK_SET_ADR] = data;

			DWIN_writeRegiser(&data, DW_UST_SICAKLIK_SET_ADR, sizeof(data));


			parse16BitTo8Bit(data, &data2[0], &data2[1]);
			EEPROM_Write(&hi2c1, DW_UST_SICAKLIK_SET_ADR, data2, sizeof(data2));


		break;

		case DW_ALT_SICAKLIK_SET_ADR:

			registerTable[DW_ALT_SICAKLIK_SET_ADR] = data;

			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_SET_ADR, sizeof(data));

			parse16BitTo8Bit(data, &data2[0], &data2[1]);
			EEPROM_Write(&hi2c1, DW_ALT_SICAKLIK_SET_ADR, data2, sizeof(data2));


		break;

		case DW_PISIRME_SURESI_ADR:

			registerTable[DW_PISIRME_SURESI_ADR] 		= data;
			registerTable[DW_PISIRME_SURESI_ORT_ADR] 	= data;

			DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			data = 0;

			DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));

			EEPROM_Write(&hi2c1, DW_PISIRME_SURESI_ADR, data2, sizeof(data2));


		break;

		case DW_BUHAR_SURESI_ADR:

			registerTable[DW_BUHAR_SURESI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			DWIN_writeRegiser(&data, DW_BUHAR_SURESI_ADR, sizeof(data));

			EEPROM_Write(&hi2c1, DW_BUHAR_SURESI_ADR, data2, sizeof(data2));

		break;

		case DW_PISIRME_BASLATMA_ADR:

			if(data == 1)
			{
				if(registerTable[DW_PISIRME_BASLATMA_ADR] != 1)
				{
					registerTable[DW_PISIRME_BASLATMA_ADR] = data;
					pisirmeManuelDownCounter = (registerTable[DW_PISIRME_SURESI_ADR] * 60) + 1;
					registerTable[DW_PISIRME_SURESI_ORT_ADR] = registerTable[DW_PISIRME_SURESI_ADR];
					DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
				}
			}

		break;

		case DW_PISIRME_SONLANDIRMA_ADR:

			if(data == 1)
			{
				registerTable[DW_PISIRME_BASLATMA_ADR] = 0;

				data = registerTable[DW_PISIRME_SURESI_ORT_ADR];
				registerTable[DW_PISIRME_SURESI_ADR] 	= registerTable[DW_PISIRME_SURESI_ORT_ADR];
				registerTable[DW_PISIRME_SURESI_SN_ADR] = 0;

				DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

				data = 0;

				DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));
				DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
				DWIN_writeRegiser(&data, DW_PISIRME_SONLANDIRMA_ADR, sizeof(data));

			}


		break;


		case DW_SURE_SONU_ALARM_ANIM_ADR:

			data = 0;

			registerTable[DW_SURE_SONU_ALARM_ANIM_ADR] = data;

			pisirmeSonuAlarmFlag = data;
			pisirmeSonuAlarmBuzzer = data;
			setOut(BUZZER, data);
			setOut(K10, data);

			registerTable[DW_PISIRME_BASLATMA_ADR] = 0;

			data = registerTable[DW_PISIRME_SURESI_ORT_ADR];
			registerTable[DW_PISIRME_SURESI_ADR] 	= registerTable[DW_PISIRME_SURESI_ORT_ADR];
			registerTable[DW_PISIRME_SURESI_SN_ADR] = 0;

			DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

			data = 0;

			DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));
			DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
			DWIN_writeRegiser(&data, DW_PISIRME_SONLANDIRMA_ADR, sizeof(data));
			DWIN_writeRegiser(&data, DW_SURE_SONU_ALARM_ANIM_ADR, sizeof(data));

			if(registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)
			{
				if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						touchSetOnOff_structure(0,95,0,5);			// Manuel - tek tc buhar yok

					else
						touchSetOnOff_structure(0,94,0,5);			// Manuel - tek tc buhar var
				}
				else
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						touchSetOnOff_structure(0,96,0,5);			// Manuel - cift tc buhar yok

					else
						touchSetOnOff_structure(0,2,0,5);			// Manuel - cift tc buhar var
				}
			}
			else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
			{
				if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						touchSetOnOff_structure(0,85,0,5);			// Recete - tek tc buhar yok

					else
						touchSetOnOff_structure(0,84,0,5);			// Recete - tek tc buhar var
				}
				else
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						touchSetOnOff_structure(0,83,0,5);			// Recete - cift tc buhar yok

					else
						touchSetOnOff_structure(0,82,0,5);			// Recete - cift tc buhar var
				}
			}

		break;

		case DW_PARAM_LAMBA_SURESI_ADR:

			registerTable[DW_PARAM_LAMBA_SURESI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_LAMBA_SURESI_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_LAMBA_SURESI_ADR, sizeof(data));

		break;


		case DW_PARAM_BUTTON_SOUND_ADR:

			registerTable[DW_PARAM_BUTTON_SOUND_ADR] = data;

			if(data == 1)
				DWIN_buzzerSet(1);
			else
				DWIN_buzzerSet(0);

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUTTON_SOUND_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_BUTTON_SOUND_ADR, sizeof(data));


		break;

		case DW_PARAM_ALARM_ADR:

			registerTable[DW_PARAM_ALARM_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_ALARM_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_ALARM_ADR, sizeof(data));

		break;

		case DW_PARAM_PSW_ADR:

			registerTable[DW_PARAM_PSW_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_PSW_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_PSW_ADR, sizeof(data));

		break;


		case DW_PARAM_BUHAR_SENSOR_TYPE_ADR:

			registerTable[DW_PARAM_BUHAR_SENSOR_TYPE_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_SENSOR_TYPE_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_BUHAR_SENSOR_TYPE_ADR, sizeof(data));

		break;


		case DW_PARAM_BUHAR_MAX_SET_ADR:

			registerTable[DW_PARAM_BUHAR_MAX_SET_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_MAX_SET_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_BUHAR_MAX_SET_ADR, sizeof(data));

		break;

		case DW_PARAM_BUHAR_HAZIR_SICAK_ADR:

			registerTable[DW_PARAM_BUHAR_HAZIR_SICAK_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_HAZIR_SICAK_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_BUHAR_HAZIR_SICAK_ADR, sizeof(data));

		break;

		case DW_PARAM_BUHAR_UST_HIS_ADR:

			registerTable[DW_PARAM_BUHAR_UST_HIS_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_UST_HIS_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_BUHAR_UST_HIS_ADR, sizeof(data));

		break;

		case DW_PARAM_BUHAR_ALT_HIS_ADR:

			registerTable[DW_PARAM_BUHAR_ALT_HIS_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_ALT_HIS_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_BUHAR_ALT_HIS_ADR, sizeof(data));

		break;

		case DW_PARAM_TERMOKUPL_TYPE_ADR:

			registerTable[DW_PARAM_TERMOKUPL_TYPE_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_TERMOKUPL_TYPE_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_TERMOKUPL_TYPE_ADR, sizeof(data));

		break;

		case DW_PARAM_LOGO_ADR:

			registerTable[DW_PARAM_LOGO_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_LOGO_ADR, data2, sizeof(data2));

			DWIN_writeRegiser(&data, DW_PARAM_LOGO_CNT_ADR, sizeof(data));

			DWIN_writeRegiser(&data, DW_PARAM_LOGO_ADR, sizeof(data));

		break;

		case DW_PARAM_DIL_ADR:

			registerTable[DW_PARAM_DIL_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_DIL_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_BUHAR_ACTIVE_ADR:

			registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_ACTIVE_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_CIHAZ_TYPE_ADR:

			registerTable[DW_PARAM_CIHAZ_TYPE_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_CIHAZ_TYPE_ADR, data2, sizeof(data2));

		break;

		case DW_PARAMETRE_EXIT_PAGE_ADR:

			uint8_t pageChangeCheck = 0;

			uint8_t readData[2];
			DWIN_readRegister(readData, DW_PARAM_DIL_ADR, sizeof(readData));

			if(readData[1] != registerTable[DW_PARAM_DIL_ADR])
			{

				DWIN_changePage(DW_EMPTY_PAGE_NUM);

				uint16_t writeData = registerTable[DW_PARAM_DIL_ADR];
				DWIN_writeRegiser(&writeData, DW_PARAM_DIL_ADR, sizeof(data));

				writeData = 1;
				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_dilChange();

				EEPROM_Recete_DefaultWrite(&hi2c1);
				EEPROM_Recete_Read(&hi2c1);

				writeData = 0;
				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_changePage(0);

			}

			DWIN_readRegister(readData, DW_PARAM_BUHAR_ACTIVE_ADR, sizeof(readData));

			if(readData[1] != registerTable[DW_PARAM_BUHAR_ACTIVE_ADR])
			{
				pageChangeCheck++;

				DWIN_changePage(DW_EMPTY_PAGE_NUM);

				uint16_t writeData = registerTable[DW_PARAM_BUHAR_ACTIVE_ADR];
				DWIN_writeRegiser(&writeData, DW_PARAM_BUHAR_ACTIVE_ADR, sizeof(data));

				writeData = 1;
				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_change_buhar_settings(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR]);

				writeData = 0;
				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_changePage(0);
			}

			DWIN_readRegister(readData, DW_PARAM_CIHAZ_TYPE_ADR, sizeof(readData));

			if(readData[1] != registerTable[DW_PARAM_CIHAZ_TYPE_ADR])
			{
				pageChangeCheck++;

				DWIN_changePage(DW_EMPTY_PAGE_NUM);

				uint16_t writeData = registerTable[DW_PARAM_CIHAZ_TYPE_ADR];
				DWIN_writeRegiser(&writeData, DW_PARAM_CIHAZ_TYPE_ADR, sizeof(data));

				writeData = 1;
				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_change_cihaz_type_settings(registerTable[DW_PARAM_CIHAZ_TYPE_ADR]);

				writeData = 0;
				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_changePage(0);
			}

			if(pageChangeCheck>0)
			{
				if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						returnkeycode_change_structure(DW_PAGE_MANUEL_BUHARYOK_TEKTC_ADR);			// Manuel - tek tc buhar yok

					else
						returnkeycode_change_structure(DW_PAGE_MANUEL_BUHARVAR_TEKTC_ADR);			// Manuel - tek tc buhar var
				}
				else
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						returnkeycode_change_structure(DW_PAGE_MANUEL_BUHARYOK_CIFTTC_ADR);			// Manuel - cift tc buhar yok

					else
						returnkeycode_change_structure(DW_PAGE_MANUEL_BUHARVAR_CIFTTC_ADR);			// Manuel - cift tc buhar var
				}
			}


		break;

		case DW_FARBRIKA_AYAR_PARAM_ADR:

			if(data == 1)
			{
				uint8_t eraseWrite = 0;
				EEPROM_Write(&hi2c1, EEPROM_USAGE_CHECK_ADDR, &eraseWrite, 1);
				HAL_Delay(0);

				DWIN_writeRegiser((uint16_t[]){0x55aa,0x5aa5},0x0004,4);

				HAL_Delay(1000);

				NVIC_SystemReset();
			}

		break;

		case (DW_FIRST_WRITE_RTC_ADR + 5):

			uint8_t saniye 	= (uint8_t)registerTable[DW_FIRST_WRITE_RTC_ADR],
					dakika 	= (uint8_t)registerTable[DW_FIRST_WRITE_RTC_ADR + 1],
					saat	= (uint8_t)registerTable[DW_FIRST_WRITE_RTC_ADR + 2],
					gun		= (uint8_t)registerTable[DW_FIRST_WRITE_RTC_ADR + 3],
					ay		= (uint8_t)registerTable[DW_FIRST_WRITE_RTC_ADR + 4],
					yil		= (uint8_t)data;

			registerTable[DW_FIRST_WRITE_RTC_ADR + 5] = data;

			DWIN_writeRTC(saniye, dakika, saat, gun, ay, yil, 1);
			RTC_SetDateTime(saat, dakika, saniye, gun, ay, yil);

		break;


		case DW_ARIZA_ALARM_SUSTURMA_ADR:

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);
			setOut(K10, 0);

		break;

		case BLE_DVC_INFO_UPDATE_ADR:

			registerTable[BLE_DVC_INFO_UPDATE_ADR] = data;
			uint8_t qrCodeData[MAX_DVC_PSW_SIZE + MFD_DATA_SIZE] = {0};

			for(int i=0;i<MFD_DATA_SIZE;i++)
				qrCodeData[i] = (uint8_t)registerTable[BLE_DVC_MFD_ADR + i];

			for(int i=0;i<MAX_DVC_PSW_SIZE;i++)
				qrCodeData[i+MFD_DATA_SIZE] = (uint8_t)registerTable[BLE_DVC_PSW_ADR + i];


			uint8_t strBuffer[(sizeof(qrCodeData) * 2) + 1] = {0};

			hexToString(qrCodeData, sizeof(qrCodeData), strBuffer);

			uint16_t qrCodeData_u16[(sizeof(strBuffer)/2) + 1] = {0};

			convert_u8_to_u16(strBuffer, qrCodeData_u16, sizeof(strBuffer));

			qrCodeData_u16[(sizeof(strBuffer)/2)] = 0xFF;

			DWIN_writeRegiser(qrCodeData_u16, DW_QR_CODE_ADR,sizeof(qrCodeData_u16));


		break;

		case BLE_DVC_CONN_ADR:


			DWIN_writeRegiser(&data, DW_BLE_IKON_ADR,sizeof(data));

			registerTable[BLE_DVC_CONN_ADR] = data;

			if(data == 0)
				registerTable[BLE_DVC_LOCK_ADR] = data;

		break;

		default:
			registerTable[addr] = data;
		break;
	}

	if((addr >= APP_OTOMATIK_ACMA_SAAT_ADR) && (addr < (APP_OTOMATIK_ACMA_SAAT_ADR + ((EE_OTOMATIK_ACMA_PARAM_SIZE/2)*7))))
		otomatikAcmaWriteCheck++;
	else if((addr >= APP_RECETE_ILK_ADR) && (addr < (APP_RECETE_ILK_ADR + (APP_RECETE_LENGTH*100))))
		receteDuzenlemeAdr = addr;

}

void Bluetooth_run(void)
{
	Bluetooth_Check();
	Bluetooth_writeRegister();
}

void set_bootloader_request_flag(void)
{
    /* PWR & BKP clock enable */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;

    /* Backup domain erişimi aç */
    PWR->CR |= PWR_CR_DBP;

    /* Magic number yaz */
    BKP->DR1 = BL_MAGIC_NUMBER;

    /* Yazma tamamlandıktan sonra reset */
}

void Jump_To_Bootloader(void)
{
	set_bootloader_request_flag();
	HAL_Delay(10);
	NVIC_SystemReset();
}


