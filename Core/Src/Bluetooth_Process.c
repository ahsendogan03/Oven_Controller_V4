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
extern uint8_t altTurbo		;

uint16_t ESP32_writeData[100];
uint16_t ESP32_writeAmountAddress = 0;
uint16_t ESP32_writeAddress = 0;

uint8_t esp32_txBuffer[255];

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

    // CRC OK - Bluetooth bağlantı zaman damgası güncelle
    counterTick.bluetoothCheck = HAL_GetTick();

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
                    SEGGER_RTT_printf(0, "  ESP32_writeData[%d] = 0x%04X\r\n", i, ESP32_writeData[i]);
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

            HAL_UART_Transmit_IT(ESP32_huart_channel, esp32_txBuffer, 8);

            #if DEBUG_ESP32 == 1
            SEGGER_RTT_printf(0, "ESP32 Write ACK sent\r\n");
            #endif

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
	if(ESP32.rxDoneFlag == 1)
	{
		ESP32.rxDoneFlag = 0;
		ESP32_receiveDataProcess();
	}

    if (HAL_GetTick() - counterTick.bluetoothCheck > BLUETOOTH_TIMEOUT_MS)
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

void Bluetooth_writeRegister(void)
{
	if(ESP32_writeAmountAddress > 0)
	{
		for(int i=0;i<ESP32_writeAmountAddress;i++)
		{
			Bluetooth_dwinWrite(ESP32_writeAddress, ESP32_writeData[i]);
		}

		ESP32_writeAmountAddress = 0;
		memset(ESP32_writeData,0,sizeof(ESP32_writeData));
	}
}

void Bluetooth_dwinWrite(uint16_t addr, uint16_t value)
{
	uint16_t data = value;
	uint8_t data2[2];

	switch(addr)
	{

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

				DWIN_changePage(2);
				setOut(K14, data);
				DWIN_enterManuelProcess();
			}

			// sayfa degistirme komutu



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

		case DW_UST_ON_SET_ADR:

			registerTable[DW_UST_ON_SET_ADR] = data;

			DWIN_writeRegiser(&data, DW_UST_ON_SET_ADR, sizeof(data));

			parse16BitTo8Bit(data, &data2[0], &data2[1]);
			EEPROM_Write(&hi2c1, DW_UST_ON_SET_ADR, data2, sizeof(data2));


		break;

		case DW_UST_ARKA_SET_ADR:

			registerTable[DW_UST_ARKA_SET_ADR] = data;

			DWIN_writeRegiser(&data, DW_UST_ARKA_SET_ADR, sizeof(data));

			parse16BitTo8Bit(data, &data2[0], &data2[1]);
			EEPROM_Write(&hi2c1, DW_UST_ARKA_SET_ADR, data2, sizeof(data2));

		break;

		case DW_ALT_SET_ADR:

			registerTable[DW_ALT_SET_ADR] = data;

			DWIN_writeRegiser(&data, DW_ALT_SET_ADR, sizeof(data));

			parse16BitTo8Bit(data, &data2[0], &data2[1]);
			EEPROM_Write(&hi2c1, DW_ALT_SET_ADR, data2, sizeof(data2));


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

				//DWIN_resetManuelPisirme();
			}

			//DWIN_resetManuelPisirme();

		break;

		case DW_PISIRME_ALARM_SUSTURMA_ADR:

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);

			registerTable[DW_PISIRME_BASLATMA_ADR] = 0;

			data = registerTable[DW_PISIRME_SURESI_ORT_ADR];
			registerTable[DW_PISIRME_SURESI_ADR] 	= registerTable[DW_PISIRME_SURESI_ORT_ADR];
			registerTable[DW_PISIRME_SURESI_SN_ADR] = 0;

			DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

			data = 0;

			DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));
			DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
			DWIN_writeRegiser(&data, DW_PISIRME_SONLANDIRMA_ADR, sizeof(data));

			//DWIN_resetManuelPisirme();

		break;

		case DW_ARIZA_ALARM_SUSTURMA_ADR:

			data = 1;

			registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR] = 1;

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);

			DWIN_writeRegiser(&data, DW_ARIZA_ALARM_SUSTURMA_ADR, sizeof(data));

		break;

		default:
			registerTable[addr] = data;
		break;
	}
}

void Bluetooth_run(void)
{
	Bluetooth_Check();
	Bluetooth_writeRegister();
}



