/*
 * DWIN_Process.c
 *
 *  Created on: Jan 20, 2025
 *      Author: Step
 */


#include "usart.h"
#include "USART_Process.h"
#include "DWIN_Process.h"
#include "SEGGER_RTT.h"
#include "Temperature_Process.h"
#include "string.h"
#include "InOut_Process.h"
#include "rtc.h"
#include "EEPROM_Process.h"
#include "PID_Control.h"
#include "hdc1080.h"

extern TemperatureData temp;
extern uint8_t DWIN_rxBuffer[DWIN_rxBufferSize];
extern uint8_t main_DWIN_rxBuffer[DWIN_rxBufferSize];

extern USART_TypeDef *DWIN_usartDeclaration;
extern UART_HandleTypeDef *DWIN_huart_channel;
extern DMA_HandleTypeDef *DWIN_hdma_usart_purpose;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern I2C_HandleTypeDef hi2c1;

extern usartInfo DWIN;

extern uint16_t eepromAddrTable[7];

extern uint8_t rxBusyFlag;

tickCounter counterTick;

volatile uint16_t registerTable[9000];


uint16_t pisirmeManuelDownCounter 	= 0;
uint16_t buharManuelDownCounter 	= 0;

uint8_t rtcSecondPisirme 	= 0;
uint8_t rtcSecondBuhar 		= 0;

uint8_t pisirmeSonuAlarmFlag = 0;
uint8_t pisirmeSonuAlarmBuzzer = 0;

uint8_t wrongmsg_flag = 0;
uint8_t wrongmsg_buffer[20];

uint8_t ustSicaklikProcess = 99;
uint8_t altSicaklikProcess = 99;

uint8_t ustOnTurbo 		= 0;
uint8_t ustArkaTurbo 	= 0;
uint8_t altTurbo		= 0;
uint8_t turboCloseFlag	= 0;

uint16_t alarmBuzzerPeriod = 1000;

uint16_t islemdekiRecete 		= 0;
uint16_t islemdekiReceteAdim 	= 1;
//uint16_t receteAdimSayisiTable[DW_RECETE_AMOUNT];

// 8 bitlik iki sayıyı 16 bitlik bir sayıya birleştiren fonksiyon
uint16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    return ((uint16_t)highByte << 8) | lowByte;
}

void parse16BitTo8Bit(uint16_t value, uint8_t *highByte, uint8_t *lowByte) {
    *highByte = (uint8_t)(value >> 8);  // Yüksek byte (MSB)
    *lowByte = (uint8_t)(value & 0xFF); // Düşük byte (LSB)
}

void convert_u8_to_u16(const uint8_t src[], uint16_t dest[], uint16_t size)
{

    for (int i = 0; i < size/2 ; i++)
    {
        dest[i] = ((uint16_t)src[2 * i] << 8) | src[2 * i + 1];
    }
}

void convert_u16_to_u8(const uint16_t src[], uint8_t dest[], uint16_t size)
{
    for (int i = 0; i < size/2 ; i++)
    {
        dest[2 * i + 1]     = (uint8_t)(src[i] & 0xFF);       // Lower byte
        dest[2 * i] = (uint8_t)((src[i] >> 8) & 0xFF);        // Higher byte
    }
}

uint16_t calculateCRC16Modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF; // Başlangıç değeri

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i]; // İlk veri byte'ı ile XOR işlemi
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // Polinom: x^16 + x^15 + x^2 + 1
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
	DWIN_manuelPisirmeSuresi_Calc();
	DWIN_manuelBuharSuresi_Calc();
}


DWIN_Response DWIN_receiveDataProcess(void)
{
	DWIN_Response response = NO_RESPONSE;

	if((DWIN_rxBuffer[0] == 0x5A) && (DWIN_rxBuffer[1] == 0xA5))
	{
		uint8_t size = DWIN_rxBuffer[2]+3;

		if((DWIN_rxBuffer[size] == 0x5A) && (DWIN_rxBuffer[size + 3] == READ_CMD))
		{
			size = DWIN_rxBuffer[size + 2] + 3;

			uint8_t secondBufferstartAdr = DWIN_rxBuffer[2]+3;

			for(int i=secondBufferstartAdr;i<secondBufferstartAdr+size;i++)
			{
				DWIN_rxBuffer[i - secondBufferstartAdr] = DWIN_rxBuffer[i];
			}

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0," write and read ! \r\n");
			#endif
		}

		uint8_t crcBuffer[(uint8_t)(DWIN_rxBuffer[2]-2)];

		for(int i=0;i<DWIN_rxBuffer[2]-2;i++)
			crcBuffer[i] = DWIN_rxBuffer[i+3];

		uint16_t crc = calculateCRC16Modbus(crcBuffer, sizeof(crcBuffer));

		uint16_t crcRx = combineBytes(DWIN_rxBuffer[size-1], DWIN_rxBuffer[size-2]);

		if(crc == crcRx)
		{
			if(DWIN_rxBuffer[3] == WRITE_CMD)
				response = WRITE_OK;

			if(DWIN_rxBuffer[3] == READ_CMD)
			{
				uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);

				if((addr != 0x979A) && (addr != 0x0084) && (addr != 0x7200))
				{
					#if DEBUG_DWIN == 1
					SEGGER_RTT_printf(0,"---------------------------\r\nAddr   : %x %x \r\n",DWIN_rxBuffer[4],DWIN_rxBuffer[5]);

					for(int i=0;i<DWIN_rxBuffer[6];i++)
						SEGGER_RTT_printf(0,"Data %d : %x %x \r\n",i+1,DWIN_rxBuffer[7+(i*2)],DWIN_rxBuffer[8+(i*2)]);

					#endif

					response = READ_OK;
				}
			}
		}

		else
		{
			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0," CRC ERROR ! \r\n");
			#endif

			response = CRC_ERROR;
		}

	}
	else
	{

		#if DEBUG_DWIN == 1
		SEGGER_RTT_printf(0," False Message ! \r\n");
		wrongmsg_flag = 1;

		for(int i=0;i<20;i++)
			wrongmsg_buffer[i] = DWIN_rxBuffer[i];

		#endif
	}

	return response;
}

HAL_StatusTypeDef DWIN_SetUsartChannel(UART_HandleTypeDef *huart, USART_TypeDef *Declaration, DMA_HandleTypeDef *hdma)
{
	HAL_StatusTypeDef response;

	DWIN_usartDeclaration = Declaration;
	DWIN_huart_channel = huart;
	DWIN_hdma_usart_purpose = hdma;


//	response = HAL_UARTEx_ReceiveToIdle_DMA(huart, DWIN_rxBuffer, DWIN_rxBufferSize);
//	__HAL_DMA_DISABLE_IT(DWIN_hdma_usart_purpose, DMA_IT_HT);

	  response = HAL_UART_Receive_DMA(huart, main_DWIN_rxBuffer, DWIN_rxBufferSize);

	//__HAL_UART_CLEAR_IT(huart, UART_CLEAR_IDLEF);
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);


	return response;
}

DWIN_Response DWIN_readRegister(uint8_t* pBuffer, uint16_t addr, uint8_t len)
{
	DWIN_Response response = NO_RESPONSE;

	uint8_t numOfRegister = len / 2;

	uint8_t txBuffer[numOfRegister + 8];

	txBuffer[0] = 0x5A;
	txBuffer[1] = 0xA5;
	txBuffer[2] = 0x06;
	txBuffer[3] = 0x83;

	uint8_t highByte, lowByte;
    highByte 	= (addr >> 8) & 0xFF;
    lowByte 	= addr & 0xFF;

	txBuffer[4] = highByte;
	txBuffer[5] = lowByte;
	txBuffer[6] = numOfRegister;

	uint8_t crcBuffer[4];

	for(int i=3;i<7;i++)
		crcBuffer[i-3] = txBuffer[i];

	uint16_t crc = calculateCRC16Modbus(crcBuffer, sizeof(crcBuffer));

	txBuffer[7] = crc & 0xFF;
	txBuffer[8] = (crc >> 8) & 0xFF;

	uint8_t numOfAttempts = 0;

	sendPoint:

	//DWIN.receiveStatus = NO_RESPONSE;
	memset(DWIN_rxBuffer,0,sizeof(DWIN_rxBuffer));

	DWIN.rxDoneFlag = 0;
	HAL_UART_Transmit_IT(DWIN_huart_channel, txBuffer, sizeof(txBuffer));
	numOfAttempts++;

	uint32_t timeOut = HAL_GetTick();
	while((DWIN.rxDoneFlag != 1)&&((HAL_GetTick() - timeOut)<=TIMEOUT_MS));

	if(((HAL_GetTick() - timeOut) <= TIMEOUT_MS)&&(numOfAttempts <= MAX_ATTEMPT))
	{
		response = DWIN_receiveDataProcess();

		if(response == READ_OK)
		{
			for(int i=0;i<(numOfRegister*2);i++)
				pBuffer[i] = DWIN_rxBuffer[i+7];

			DWIN.rxDoneFlag = 0;
		}
		else
		{
			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0,"DWIN CRC ERROR \r\n");
			#endif

			goto sendPoint;
		}

	}

	else
	{
		if(numOfAttempts <= MAX_ATTEMPT)
		{
			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0,"DWIN NO RESPONSE \r\n");
			#endif

			goto sendPoint;
		}

		DWIN.rxDoneFlag = 0;
		DWIN.Init = 0;
	}

	return response;

}

void changeMaxSetValue(uint16_t maxValue)
{
	HAL_Delay(20);

	for(int i=0;i<3;i++)
	{
		uint8_t txBuffer[48];
		uint16_t addr = 0x00B0;

		txBuffer[0] = 0x5A;
		txBuffer[1] = 0xA5;
		txBuffer[2] = 0x2D;
		txBuffer[3] = 0x82;

		uint8_t highByte, lowByte;
		highByte 	= (addr >> 8) & 0xFF;
		lowByte 	= addr & 0xFF;

		txBuffer[4] = highByte;
		txBuffer[5] = lowByte;

		txBuffer[6] 	= 0x5A;
		txBuffer[7] 	= 0xA5;

		txBuffer[8] 	= 0x00;
		txBuffer[9] 	= 0x05+i;

		txBuffer[10] 	= 0x01;
		txBuffer[11] 	= 0x02;

		txBuffer[12] 	= 0x00;
		txBuffer[13] 	= 0x03;

		txBuffer[14] 	= 0x00;
		txBuffer[15] 	= 0x05+i;

		txBuffer[16] 	= 0x02;
		txBuffer[17] 	= 0x04;

		txBuffer[18] 	= 0x02;
		txBuffer[19] 	= 0x44;

		txBuffer[20] 	= 0x02;
		txBuffer[21] 	= 0x7F;

		txBuffer[22] 	= 0x02;
		txBuffer[23] 	= 0xC7;

		txBuffer[24] 	= 0xFF;
		txBuffer[25] 	= 0x00;

		txBuffer[26] 	= 0xFF;
		txBuffer[27] 	= 0x00;

		txBuffer[28] 	= 0xFD;
		txBuffer[29] 	= 0x02;

		txBuffer[30] 	= 0xFE;

		txBuffer[31] 	= 0x15;
		txBuffer[32] 	= 0x5F+(i*2);

		txBuffer[33] 	= 0x00;

		txBuffer[34] 	= 0x01;

		txBuffer[35] 	= 0x01;

		txBuffer[36] 	= 0x00;
		txBuffer[37] 	= 0x01;

		txBuffer[38] 	= 0x00;
		txBuffer[39] 	= 0x00;

		highByte 		= (maxValue >> 8) & 0xFF;
		lowByte 		= maxValue & 0xFF;

		txBuffer[40] 	= highByte;
		txBuffer[41] 	= lowByte;

		txBuffer[42] 	= 0x00;
		txBuffer[43] 	= 0x00;
		txBuffer[44] 	= 0x00;
		txBuffer[45] 	= 0x00;

		uint8_t crcBuffer[43];

		for(int i=3;i<46;i++)
			crcBuffer[i-3] = txBuffer[i];

		uint16_t crc = calculateCRC16Modbus(crcBuffer, sizeof(crcBuffer));

		txBuffer[46] = crc & 0xFF;
		txBuffer[47] = (crc >> 8) & 0xFF;


		HAL_UART_Transmit_IT(DWIN_huart_channel, txBuffer, sizeof(txBuffer));

		HAL_Delay(40);

	}

	for(int i=0;i<3;i++)
	{
		uint8_t txBuffer[48];
		uint16_t addr = 0x00B0;

		txBuffer[0] = 0x5A;
		txBuffer[1] = 0xA5;
		txBuffer[2] = 0x2D;
		txBuffer[3] = 0x82;

		uint8_t highByte, lowByte;
		highByte 	= (addr >> 8) & 0xFF;
		lowByte 	= addr & 0xFF;

		txBuffer[4] = highByte;
		txBuffer[5] = lowByte;

		txBuffer[6] 	= 0x5A;
		txBuffer[7] 	= 0xA5;

		txBuffer[8] 	= 0x00;
		txBuffer[9] 	= 0x05+i;

		txBuffer[10] 	= 0x00;
		txBuffer[11] 	= 0x02;

		txBuffer[12] 	= 0x00;
		txBuffer[13] 	= 0x03;

		txBuffer[14] 	= 0x00;
		txBuffer[15] 	= 0x05+i;

		txBuffer[16] 	= 0x00;
		txBuffer[17] 	= 0xA7;

		txBuffer[18] 	= 0x02;
		txBuffer[19] 	= 0x45;

		txBuffer[20] 	= 0x01;
		txBuffer[21] 	= 0x22;

		txBuffer[22] 	= 0x02;
		txBuffer[23] 	= 0xC8;

		txBuffer[24] 	= 0xFF;
		txBuffer[25] 	= 0x00;

		txBuffer[26] 	= 0xFF;
		txBuffer[27] 	= 0x00;

		txBuffer[28] 	= 0xFD;
		txBuffer[29] 	= 0x02;

		txBuffer[30] 	= 0xFE;

		txBuffer[31] 	= 0x15;
		txBuffer[32] 	= 0x5F+(i*2);

		txBuffer[33] 	= 0x00;

		txBuffer[34] 	= 0x00;

		txBuffer[35] 	= 0x01;

		txBuffer[36] 	= 0x00;
		txBuffer[37] 	= 0x01;

		txBuffer[38] 	= 0x00;
		txBuffer[39] 	= 0x00;

		highByte 		= (maxValue >> 8) & 0xFF;
		lowByte 		= maxValue & 0xFF;

		txBuffer[40] 	= highByte;
		txBuffer[41] 	= lowByte;

		txBuffer[42] 	= 0x00;
		txBuffer[43] 	= 0x00;
		txBuffer[44] 	= 0x00;
		txBuffer[45] 	= 0x00;

		uint8_t crcBuffer[43];

		for(int i=3;i<46;i++)
			crcBuffer[i-3] = txBuffer[i];

		uint16_t crc = calculateCRC16Modbus(crcBuffer, sizeof(crcBuffer));

		txBuffer[46] = crc & 0xFF;
		txBuffer[47] = (crc >> 8) & 0xFF;


		HAL_UART_Transmit_IT(DWIN_huart_channel, txBuffer, sizeof(txBuffer));

		HAL_Delay(40);

	}

	memset(DWIN_rxBuffer,0,sizeof(DWIN_rxBuffer));
	DWIN.rxDoneFlag = 0;

}

DWIN_Response DWIN_writeRegiser(uint16_t* pBuffer, uint16_t addr, uint8_t len)
{
	DWIN_Response response = NO_RESPONSE;

	uint8_t numOfRegister = len / 2;

	uint8_t txBuffer[(numOfRegister*2) + 8];


	txBuffer[0] = 0x5A;
	txBuffer[1] = 0xA5;
	txBuffer[2] = 0x05 + (numOfRegister*2);
	txBuffer[3] = 0x82;

	uint8_t highByte, lowByte;
    highByte 	= (addr >> 8) & 0xFF;
    lowByte 	= addr & 0xFF;

	txBuffer[4] = highByte;
	txBuffer[5] = lowByte;

	for(int i=0;i<numOfRegister;i++)
	{
		uint16_t writeData = pBuffer[i];

		uint8_t highByte, lowByte;

	    highByte 	= (writeData >> 8) & 0xFF;
	    lowByte 	= writeData & 0xFF;

		txBuffer[6+(i*2)] = highByte;
		txBuffer[7+(i*2)] = lowByte;

	}

	uint8_t crcBuffer[3+(numOfRegister*2)];

	for(int i=3;i<6+(numOfRegister*2);i++)
		crcBuffer[i-3] = txBuffer[i];


	uint16_t crc = calculateCRC16Modbus(crcBuffer,3+(numOfRegister*2));

	txBuffer[8+((numOfRegister-1)*2)] = crc & 0xFF;
	txBuffer[9+((numOfRegister-1)*2)] = (crc >> 8) & 0xFF;

	uint8_t numOfAttempts = 0;

	sendPoint:

	if(DWIN.rxDoneFlag != 1)
	{
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) == 0)
		{
			if (huart3.gState == HAL_UART_STATE_READY)
			{

				memset(DWIN_rxBuffer,0,sizeof(DWIN_rxBuffer));

				DWIN.rxDoneFlag = 0;
				HAL_UART_Transmit_IT(DWIN_huart_channel, txBuffer, sizeof(txBuffer));
				numOfAttempts++;
			}
		}


		uint32_t timeOut = HAL_GetTick();
		while((DWIN.rxDoneFlag != 1)&&((HAL_GetTick() - timeOut)<=TIMEOUT_MS));

		if(((HAL_GetTick() - timeOut) <= TIMEOUT_MS)&&(numOfAttempts <= MAX_ATTEMPT))
		{
			response = DWIN_receiveDataProcess();

			if(response == WRITE_OK)
				DWIN.rxDoneFlag = 0;

			else if(response != READ_OK)
			{
				#if DEBUG_DWIN == 1
				SEGGER_RTT_printf(0,"DWIN WRITE ERROR \r\n");
				#endif


				goto sendPoint;
			}
		}
		else
		{
			if(numOfAttempts <= MAX_ATTEMPT)
			{
				#if DEBUG_DWIN == 1
				SEGGER_RTT_printf(0,"DWIN NO RESPONSE \r\n");
				#endif

				goto sendPoint;
			}

			DWIN.Init = 0;
			DWIN.rxDoneFlag = 0;
		}

	}


	return response;
}

DWIN_Response DWIN_changePage(uint8_t pageNumber)
{
	DWIN_Response response = NO_RESPONSE;

	uint8_t txBuffer[12];

	txBuffer[0] = 0x5A;
	txBuffer[1] = 0xA5;
	txBuffer[2] = 0x09;
	txBuffer[3] = 0x82;
	txBuffer[4] = 0x00;
	txBuffer[5] = 0x84;
	txBuffer[6] = 0x5A;
	txBuffer[7] = 0x01;

	uint8_t highByte, lowByte;
    highByte 	= (pageNumber >> 8) & 0xFF;
    lowByte 	= pageNumber & 0xFF;

	txBuffer[8] = highByte;
	txBuffer[9] = lowByte;

	uint8_t crcBuffer[7];

	for(int i=3;i<10;i++)
		crcBuffer[i-3] = txBuffer[i];

	uint16_t crc = calculateCRC16Modbus(crcBuffer,7);

	txBuffer[10] = crc & 0xFF;
	txBuffer[11] = (crc >> 8) & 0xFF;

	uint8_t numOfAttempts = 0;

	sendPoint:

	//DWIN.receiveStatus = NO_RESPONSE;
	memset(DWIN_rxBuffer,0,sizeof(DWIN_rxBuffer));

	DWIN.rxDoneFlag = 0;
	HAL_UART_Transmit(DWIN_huart_channel, txBuffer, sizeof(txBuffer), 100);
	numOfAttempts++;

	uint32_t timeOut = HAL_GetTick();
	while((DWIN.rxDoneFlag != 1)&&((HAL_GetTick() - timeOut)<=TIMEOUT_MS));

	if(((HAL_GetTick() - timeOut) <= TIMEOUT_MS)&&(numOfAttempts <= MAX_ATTEMPT))
	{
		response = DWIN_receiveDataProcess();

		if(response == WRITE_OK)
			DWIN.rxDoneFlag = 0;

		else if(response != READ_OK)
		{
			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0,"DWIN WRITE ERROR \r\n");
			#endif


			goto sendPoint;
		}
	}
	else
	{
		if(numOfAttempts <= MAX_ATTEMPT)
		{
			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0,"DWIN NO RESPONSE \r\n");
			#endif

			goto sendPoint;
		}

		DWIN.Init = 0;
		DWIN.rxDoneFlag = 0;
	}

	return response;

}


void DWIN_run(void)
{
	if((HAL_GetTick() - counterTick.run) >= 1000)
	{
		counterTick.run = HAL_GetTick();

		HAL_GPIO_TogglePin(RUN_LED);

		if(DWIN.Init == 1)
		{
			calculate_temperature();

			registerTable[DW_MCP9700_ADR] 		= temp.TMP;
			registerTable[DW_UST_SICAKLIK_ADR] 	= temp.TC3;
			registerTable[DW_ALT_SICAKLIK_ADR] 	= temp.TC2;

			uint16_t sendData[3] = {(uint16_t)temp.TMP,temp.TC3,temp.TC2};
			DWIN_writeRegiser(sendData, DW_MCP9700_ADR,sizeof(sendData));

			if((registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)||(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER))
			{
				DWIN_arızaCheck();

				if(registerTable[DW_ARIZA_PAGE_ADR] != 1)
					PID_Run();



			}

		}

	}

	if((HAL_GetTick() - counterTick.shiftRefreshWait) >= 5)
	{
		counterTick.shiftRefreshWait = HAL_GetTick();
		pwmOutProcess();
	}

	#if DEBUG_DWIN == 1
	if(wrongmsg_flag == 1)
	{
		SEGGER_RTT_printf(0,"wrong msg : ");

		for(int i=0;i<20;i++)
		{
			HAL_Delay(0);
			SEGGER_RTT_printf(0," %x ",wrongmsg_buffer[i]);
		}

		SEGGER_RTT_printf(0,"\r\n");

		wrongmsg_flag = 0;
		memset(wrongmsg_buffer,0,sizeof(wrongmsg_buffer));
	}
	#endif

	DWIN_check();
	DWIN_answerProcess();
	DWIN_manuelPisirmeSuresi();
	DWIN_manuelBuharSuresi();
	DWIN_pisirmeSonuAlarm();
	DWIN_lambaSuresi();
	DWIN_manuelPeriodProcess();
	DWIN_buharHazirCheck();
}
void DWIN_manuelPeriodProcess(void)
{
	if(ustSicaklikProcess == 1)
	{
		uint32_t period 	= registerTable[DW_ISITICI_PERIOD_ADR] * 1000;
		uint32_t ustOnSet 	= registerTable[DW_UST_ON_SET_ADR]*1000;
		uint32_t ustArkaSet = registerTable[DW_UST_ARKA_SET_ADR]*1000;

		if(ustOnTurbo != 1)
		{
			if(((HAL_GetTick() - counterTick.ustOnPeriod) >= ustOnSet) && (registerTable[DW_UST_ON_ANIM] == 1))
			{
				uint16_t data = 0;

				setOut(K1, data);
				registerTable[DW_UST_ON_ANIM] = data;
				DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));
			}
			if(((HAL_GetTick() - counterTick.ustOnPeriod) >= period) && (registerTable[DW_UST_ON_ANIM] == 0))
			{
				uint16_t data = 1;

				setOut(K1, data);
				registerTable[DW_UST_ON_ANIM] = data;
				registerTable[DW_UST_SICAKLIK_ANIM] = data;

				DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));

				counterTick.ustOnPeriod = HAL_GetTick();
			}
		}

		if(ustArkaTurbo != 1)
		{
			if(((HAL_GetTick() - counterTick.ustArkaPeriod) >= ustArkaSet) && (registerTable[DW_UST_ARKA_ANIM] == 1))
			{
				uint16_t data = 0;

				setOut(K3, data);
				registerTable[DW_UST_ARKA_ANIM] = data;
				DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));
			}
			if(((HAL_GetTick() - counterTick.ustArkaPeriod) >= period) && (registerTable[DW_UST_ARKA_ANIM] == 0))
			{
				uint16_t data = 1;

				setOut(K3, data);
				registerTable[DW_UST_ARKA_ANIM] = data;
				registerTable[DW_UST_SICAKLIK_ANIM] = data;

				DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));

				counterTick.ustArkaPeriod = HAL_GetTick();
			}
		}

		if((registerTable[DW_UST_ARKA_ANIM] == 0) && (registerTable[DW_UST_ON_ANIM] == 0) && (registerTable[DW_UST_SICAKLIK_ANIM] == 1))
		{
			uint16_t data = 0;

			registerTable[DW_UST_SICAKLIK_ANIM] = data;
			DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
		}
	}

	if((altSicaklikProcess == 1)&&(altTurbo != 1))
	{
		uint32_t period 	= registerTable[DW_ISITICI_PERIOD_ADR] * 1000;
		uint32_t altSet		= registerTable[DW_ALT_SET_ADR]*1000;

		if(((HAL_GetTick() - counterTick.altPeriod) >= altSet) && (registerTable[DW_ALT_ANIM] == 1))
		{
			uint16_t data = 0;

			setOut(K5|K6, data);
			registerTable[DW_ALT_ANIM] = data;
			registerTable[DW_ALT_SICAKLIK_ANIM] = data;

			DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
		}

		if(((HAL_GetTick() - counterTick.altPeriod) >= period) && (registerTable[DW_ALT_ANIM] == 0))
		{
			uint16_t data = 1;

			setOut(K5|K6, data);
			registerTable[DW_ALT_ANIM] = data;
			registerTable[DW_ALT_SICAKLIK_ANIM] = data;

			DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));

			counterTick.altPeriod = HAL_GetTick();
		}
	}
}


void DWIN_manuelTurboProcess(void)
{
	if(registerTable[DW_TURBO_ADR] == 1)
	{
		uint16_t ustSicaklikSet 	= registerTable[DW_UST_SICAKLIK_SET_ADR];
		uint16_t altSicaklikSet 	= registerTable[DW_ALT_SICAKLIK_SET_ADR];
		uint16_t ustSicaklik 		= registerTable[DW_UST_SICAKLIK_ADR];
		uint16_t altSicaklik 		= registerTable[DW_ALT_SICAKLIK_ADR];
		uint16_t ustOnIsiticiBand 	= registerTable[DW_UST_ON_ISITICI_BANDI_ADR];
		uint16_t ustArkaIsiticiBand = registerTable[DW_UST_ARKA_ISITICI_BANDI_ADR];
		uint16_t altIsiticiBand		= registerTable[DW_ALT_ISITICI_BANDI_ADR];
		//int16_t isiticiUstHis		= registerTable[DW_ISITICI_UST_HIS_ADR];
		int16_t isiticiAltHis		= registerTable[DW_ISITICI_ALT_HIS_ADR];

		uint16_t data;

		if(((altIsiticiBand + altSicaklik) < altSicaklikSet) && (altTurbo != 1))
		{
			altTurbo = 1;
			turboCloseFlag = 0;

			data = 1;

			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));

			registerTable[DW_ALT_SICAKLIK_ANIM] = data;
			registerTable[DW_ALT_ANIM] = data;

			setOut(K6|K5, data);

			SEGGER_RTT_printf(0,"Alt Turbo Aktif \r\n");

			if(((ustArkaIsiticiBand + ustSicaklik) < ustSicaklikSet) && (ustArkaTurbo != 1))
			{
				ustArkaTurbo = 1;
				turboCloseFlag = 0;

				data = 1;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));

				registerTable[DW_UST_SICAKLIK_ANIM] = data;
				registerTable[DW_UST_ARKA_ANIM] = data;

				setOut(K3, data);

				SEGGER_RTT_printf(0,"Ust Arka Turbo Aktif \r\n");
			}

			if(((ustOnIsiticiBand + ustSicaklik) < ustSicaklikSet) && (ustOnTurbo != 1))
			{
				ustOnTurbo = 1;
				turboCloseFlag = 0;

				data = 1;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));

				registerTable[DW_UST_SICAKLIK_ANIM] = data;
				registerTable[DW_UST_ON_ANIM] = data;

				setOut(K1, data);

				SEGGER_RTT_printf(0,"Ust On Turbo Aktif \r\n");
			}
		}

		if(altTurbo == 1)
		{
			if(((ustOnIsiticiBand + ustSicaklik) < ustSicaklikSet) && (ustOnTurbo != 1))
			{
				ustOnTurbo = 1;
				turboCloseFlag = 0;

				data = 1;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));

				registerTable[DW_UST_SICAKLIK_ANIM] = data;
				registerTable[DW_UST_ON_ANIM] = data;

				setOut(K1, data);

				SEGGER_RTT_printf(0,"Ust On Turbo Aktif \r\n");
			}

			if(((ustArkaIsiticiBand + ustSicaklik) < ustSicaklikSet) && (ustArkaTurbo != 1))
			{
				ustArkaTurbo = 1;
				turboCloseFlag = 0;

				data = 1;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));

				registerTable[DW_UST_SICAKLIK_ANIM] = data;
				registerTable[DW_UST_ARKA_ANIM] = data;

				setOut(K3, data);

				SEGGER_RTT_printf(0,"Ust Arka Turbo Aktif \r\n");
			}

			if((altIsiticiBand + altSicaklik) >= altSicaklikSet)
			{
				altTurbo = 0;
				ustOnTurbo = 0;
				ustArkaTurbo = 0;

				data = 0;

				DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));

				registerTable[DW_ALT_SICAKLIK_ANIM] = data;
				registerTable[DW_ALT_ANIM] = data;

				setOut(K6|K5, data);

				SEGGER_RTT_printf(0,"Alt Turbo Kapali \r\n");

				counterTick.altPeriod 	= HAL_GetTick();
			}

		}

		if((ustOnTurbo == 1)&&((ustOnIsiticiBand + ustSicaklik) >= ustSicaklikSet))
		{
			ustOnTurbo = 0;
			data = 0;

			DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));

			registerTable[DW_UST_ON_ANIM] = data;

			setOut(K1, data);

			SEGGER_RTT_printf(0,"Ust On Turbo Kapali \r\n");
		}

		if((ustArkaTurbo == 1)&&((ustArkaIsiticiBand + ustSicaklik) >= ustSicaklikSet))
		{
			ustArkaTurbo = 0;
			data = 0;

			DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));

			registerTable[DW_UST_ARKA_ANIM] = data;

			setOut(K3, data);

			SEGGER_RTT_printf(0,"Ust Arka Turbo Kapali \r\n");
		}

		if(altSicaklikSet <= (altSicaklik + isiticiAltHis))
		{
			if(((ustOnIsiticiBand + ustSicaklik) < ustSicaklikSet) && (ustOnTurbo != 1))
			{
				ustOnTurbo = 1;
				turboCloseFlag = 0;

				data = 1;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));

				registerTable[DW_UST_SICAKLIK_ANIM] = data;
				registerTable[DW_UST_ON_ANIM] = data;

				setOut(K1, data);

				SEGGER_RTT_printf(0,"Ust On Turbo Aktif \r\n");
			}

			if(((ustArkaIsiticiBand + ustSicaklik) < ustSicaklikSet) && (ustArkaTurbo != 1))
			{
				ustArkaTurbo = 1;
				turboCloseFlag = 0;

				data = 1;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
				DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));

				registerTable[DW_UST_SICAKLIK_ANIM] = data;
				registerTable[DW_UST_ARKA_ANIM] = data;

				setOut(K3, data);

				SEGGER_RTT_printf(0,"Ust Arka Turbo Aktif \r\n");
			}
		}
		else if(((ustArkaTurbo == 1)||(ustOnTurbo == 1))&&(altTurbo == 0))
		{
			ustOnTurbo = 0;
			ustArkaTurbo = 0;
		}

		if((registerTable[DW_UST_ARKA_ANIM] == 0)&&(registerTable[DW_UST_ON_ANIM] == 0)&&(registerTable[DW_UST_SICAKLIK_ANIM] == 1))
		{
			data = 0;
			DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
			registerTable[DW_UST_SICAKLIK_ANIM] = data;
		}

		if((altTurbo == 0)&&(ustArkaTurbo == 0)&&(ustOnTurbo == 0)&&(turboCloseFlag == 0))
		{
			counterTick.turboCloseWait = HAL_GetTick();
			turboCloseFlag = 1;
		}

		if((turboCloseFlag == 1)&&((HAL_GetTick() - counterTick.turboCloseWait) >= 10000))
		{
			data = 0;

			turboCloseFlag = 0;

			registerTable[DW_TURBO_ADR] = data;
			DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));
		}
	}
}
void DWIN_manuelProcess(void)
{
	uint16_t ustSicaklikSet 	= registerTable[DW_UST_SICAKLIK_SET_ADR];
	uint16_t altSicaklikSet 	= registerTable[DW_ALT_SICAKLIK_SET_ADR];
	uint16_t ustSicaklik 		= registerTable[DW_UST_SICAKLIK_ADR];
	uint16_t altSicaklik 		= registerTable[DW_ALT_SICAKLIK_ADR];
	//uint16_t ustOnIsiticiBand 	= registerTable[DW_UST_ON_ISITICI_BANDI_ADR];
	//uint16_t ustArkaIsiticiBand = registerTable[DW_UST_ARKA_ISITICI_BANDI_ADR];
	//uint16_t altIsiticiBand		= registerTable[DW_ALT_ISITICI_BANDI_ADR];
	int16_t isiticiUstHis		= registerTable[DW_ISITICI_UST_HIS_ADR];
	int16_t isiticiAltHis		= registerTable[DW_ISITICI_ALT_HIS_ADR];

	uint16_t data;

	if(ustSicaklikSet > (ustSicaklik + isiticiUstHis))
	{
		if(ustSicaklikProcess != 1)
		{
			data = 1;

			DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));

			registerTable[DW_UST_SICAKLIK_ANIM] = data;
			registerTable[DW_UST_ON_ANIM] 		= data;
			registerTable[DW_UST_ARKA_ANIM] 	= data;


			setOut(K3|K1, data);

			counterTick.ustOnPeriod 	= HAL_GetTick();
			counterTick.ustArkaPeriod 	= HAL_GetTick();

			ustSicaklikProcess = 1;

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0," ustSicaklikProcess = 1 \r\n");
			#endif
		}
	}
	else
	{
		if(ustSicaklikProcess != 99)
		{
			data = 0;

			DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));

			registerTable[DW_UST_SICAKLIK_ANIM] = data;
			registerTable[DW_UST_ON_ANIM] = data;
			registerTable[DW_UST_ARKA_ANIM] = data;

			setOut(K3|K1, data);

			ustSicaklikProcess = 99;

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0," ustSicaklikProcess = 99 \r\n");
			#endif
		}
	}

	if(altSicaklikSet > (altSicaklik + isiticiAltHis))
	{
		if(altSicaklikProcess != 1)
		{
			data = 1;

			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));

			registerTable[DW_ALT_SICAKLIK_ANIM] = data;
			registerTable[DW_ALT_ANIM] = data;

			setOut(K6|K5, 1);

			altSicaklikProcess = 1;

			counterTick.altPeriod 	= HAL_GetTick();

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0," altSicaklikProcess = 1 \r\n");
			#endif
		}
	}
	else
	{
		if(altSicaklikProcess != 99)
		{
			data = 0;

			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
			DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));

			registerTable[DW_ALT_SICAKLIK_ANIM] = data;
			registerTable[DW_ALT_ANIM] = data;

			setOut(K6|K5, data);

			altSicaklikProcess = 99;

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0," altSicaklikProcess = 99 \r\n");
			#endif
		}
	}

}
void DWIN_enterManuelProcess(void)
{
	uint16_t ustSicaklikSet 	= registerTable[DW_UST_SICAKLIK_SET_ADR];
	uint16_t altSicaklikSet 	= registerTable[DW_ALT_SICAKLIK_SET_ADR];
	uint16_t ustSicaklik 		= registerTable[DW_UST_SICAKLIK_ADR];
	uint16_t altSicaklik 		= registerTable[DW_ALT_SICAKLIK_ADR];
	//uint16_t ustOnIsiticiBand 	= registerTable[DW_UST_ON_ISITICI_BANDI_ADR];
	//uint16_t ustArkaIsiticiBand = registerTable[DW_UST_ARKA_ISITICI_BANDI_ADR];
	uint16_t altIsiticiBand		= registerTable[DW_ALT_ISITICI_BANDI_ADR];
	int8_t isiticiUstHis		= registerTable[DW_ISITICI_UST_HIS_ADR];
	int8_t isiticiAltHis		= registerTable[DW_ISITICI_ALT_HIS_ADR];

	uint16_t data = 1;

	if(ustSicaklikSet > (ustSicaklik + isiticiUstHis))
	{
		if(altSicaklikSet <= (altSicaklik + isiticiAltHis))
		{
			registerTable[DW_TURBO_ADR] = 1;

			DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0,"Turbo Aktif \r\n");
			#endif
		}
	}
	if(altSicaklikSet > (altSicaklik + isiticiAltHis))
	{

		if(altSicaklikSet > (altSicaklik + altIsiticiBand))
		{

			registerTable[DW_TURBO_ADR] = 1;

			DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));

			#if DEBUG_DWIN == 1
			SEGGER_RTT_printf(0,"Turbo Aktif \r\n");
			#endif
		}
	}
}

void DWIN_check(void)
{
	if((HAL_GetTick() - counterTick.dwinCheck) >= 500)
	{
		if(DWIN.Init != 1)
		{
			uint8_t version[2];

			if(DWIN_readRegister(version, VERSION_ADDR, sizeof(version)) == READ_OK)
			{
				SEGGER_RTT_printf(0,"DWIN OK ! Version : %x%x\r\n",version[0],version[1]);

				DWIN_changePage(0);

				for(int i=0;i<sizeof(eepromAddrTable)/2;i++)
				{
					uint16_t writeDwin = registerTable[eepromAddrTable[i]];
					DWIN_writeRegiser(&writeDwin, eepromAddrTable[i], sizeof(writeDwin));
				}

				changeMaxSetValue(registerTable[DW_ISITICI_PERIOD_ADR]);

				DWIN_resetManuelPisirme();

				DWIN.Init = 1;
			}

			else
			{

		    	HAL_UART_DMAStop(DWIN_huart_channel);

				if (HAL_UART_DeInit(&huart3) != HAL_OK)
				{
					SEGGER_RTT_printf(0,"DeInit ERROR !!! \r\n");
					Error_Handler();
				}

				HAL_Delay(0);

				if (HAL_UART_Init(&huart3) != HAL_OK)
				{
					SEGGER_RTT_printf(0,"Init ERROR !!! \r\n");
					Error_Handler();
				}

				if(DWIN_SetUsartChannel(&huart3, USART3, &hdma_usart3_rx) != HAL_OK)
				{
				  SEGGER_RTT_printf(0,"DWIN Set Usart Channel Error ! \r\n");
				  Error_Handler();
				}
			}
		}

		counterTick.dwinCheck = HAL_GetTick();
	}
}

void DWIN_manuelPisirmeSuresi_Calc(void)
{
	if((registerTable[DW_PISIRME_BASLATMA_ADR] == 1)&&(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		pisirmeManuelDownCounter--;

		uint16_t saniye = pisirmeManuelDownCounter % 60;
		uint16_t dakika = pisirmeManuelDownCounter / 60;

		registerTable[DW_PISIRME_SURESI_SN_ADR] = saniye;
		registerTable[DW_PISIRME_SURESI_ADR] 	= dakika;
	}
}


uint16_t pisirmeManuelDownCounter_eski = 0;
void DWIN_manuelPisirmeSuresi(void)
{
	if((registerTable[DW_PISIRME_BASLATMA_ADR] == 1)&&(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{

		if(pisirmeManuelDownCounter_eski != pisirmeManuelDownCounter)
		{
			pisirmeManuelDownCounter_eski = pisirmeManuelDownCounter;

			uint16_t saniye = registerTable[DW_PISIRME_SURESI_SN_ADR];
			uint16_t dakika = registerTable[DW_PISIRME_SURESI_ADR];

			DWIN_writeRegiser(&saniye, DW_PISIRME_SURESI_SN_ADR, sizeof(saniye));
			DWIN_writeRegiser(&dakika, DW_PISIRME_SURESI_ADR, sizeof(saniye));

			if(pisirmeManuelDownCounter <= 0)
			{
				pisirmeSonuAlarmFlag = 1;
				setOut(K10, 1);
				pisirmeSonuAlarmBuzzer = 0;
				registerTable[DW_PISIRME_BASLATMA_ADR] = 0;
				registerTable[MANUEL_SURE_SONU_ADR] = 1;

				if(registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)
					DWIN_changePage(MANUEL_SURE_SONU_ADR);
				else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
					DWIN_changePage(RECETE_SURE_SONU_ADR);
			}

			DWIN_recetePisirmeAdimProcess();
		}
	}



}

void DWIN_manuelBuharSuresi_Calc(void)
{
	if((registerTable[DW_BUHAR_PUSKURTME_ADR] == 1)&&(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		buharManuelDownCounter--;

		uint16_t saniye = buharManuelDownCounter;

		registerTable[DW_BUHAR_SURESI_ADR] = saniye;
	}
}

uint16_t buharManuelDownCounter_eski = 0;
void DWIN_manuelBuharSuresi(void)
{
	if((registerTable[DW_BUHAR_PUSKURTME_ADR] == 1)&&(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		if(buharManuelDownCounter_eski != buharManuelDownCounter)
		{
			buharManuelDownCounter_eski = buharManuelDownCounter;

			uint16_t saniye = registerTable[DW_BUHAR_SURESI_ADR];
			DWIN_writeRegiser(&saniye, DW_BUHAR_SURESI_ADR, sizeof(saniye));

			if(buharManuelDownCounter <= 0)
			{
				registerTable[DW_BUHAR_SURESI_ADR] 	= registerTable[DW_BUHAR_SURESI_ORT_ADR];
				registerTable[DW_BUHAR_PUSKURTME_ADR] = 0;
				uint16_t data = registerTable[DW_BUHAR_SURESI_ORT_ADR];
				DWIN_writeRegiser(&data, DW_BUHAR_SURESI_ADR, sizeof(data));

				data = 0;
				DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));

				setOut(K9, 0);
			}

		}
	}

}

void DWIN_lambaSuresi(void)
{
	if(registerTable[DW_LAMBA_ADR] == 1)
	{
		if((HAL_GetTick() - counterTick.lambaSuresi) >= (registerTable[DW_LAMBA_SURESI_ADR] * 1000))
		{
			setOut(K12, 0);
			setOut(K13, 0);

			registerTable[DW_LAMBA_ADR] = 0;

			uint16_t data = 0;

			DWIN_writeRegiser(&data, DW_LAMBA_ADR, sizeof(data));
		}
	}
}

void DWIN_pisirmeSonuAlarm(void)
{
	if(pisirmeSonuAlarmFlag)
	{
		if((HAL_GetTick() - counterTick.pisirmeSonuAlarm) >= alarmBuzzerPeriod)
		{
			counterTick.pisirmeSonuAlarm = HAL_GetTick();

			pisirmeSonuAlarmBuzzer = !pisirmeSonuAlarmBuzzer;
			setOut(BUZZER, pisirmeSonuAlarmBuzzer);

		}
	}
}


void DWIN_answerProcess(void)
{
	if((DWIN.Init) && (DWIN.rxDoneFlag == 1))
	{
		DWIN.rxDoneFlag = 0;

		if(DWIN_receiveDataProcess() == READ_OK)
		{
			DWIN_anaSayfa();

			if((registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)||(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER))
				DWIN_manuelSayfa();


			else if((registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_SAYFA_ENTER)||
					(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)||
					(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_DUZEN_SAYFA_ENTER))
				DWIN_receteSayfa();
		}

	}
}

void DWIN_anaSayfa(void)
{
	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);
	uint16_t data;
	uint8_t data2[2];

	switch(addr)
	{
		case DW_LAMBA_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);
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


		break;

		case DW_MANUEL_MOD_GIRIS_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 0)
			{
				registerTable[REG_DW_MODE_INFO_ADR] = DW_ANA_SAYFA_ENTER;

				DWIN_resetManuelPisirme();
//				TIM15->CCR2 = 0;
//				TIM15->CCR1 = 0;
//				TIM3->CCR1 = 0;
			}


			else if(data == 1)
			{
				registerTable[REG_DW_MODE_INFO_ADR] = DW_MANUEL_MODE_ENTER;

				setOut(K14, data);
				//DWIN_enterManuelProcess();
				PID_Setup();

			}



		break;

		case DW_RECETE_SAYFA_ENTER_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 0)
				registerTable[REG_DW_MODE_INFO_ADR] = DW_ANA_SAYFA_ENTER;

			if(data == 1)
				registerTable[REG_DW_MODE_INFO_ADR] = DW_RECETE_SAYFA_ENTER;


		break;

		case DW_PARAMETRE_PAGE_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == DW_PARAMETRE_PSW)
			{
				data = registerTable[DW_LAMBA_SURESI_ADR];
				DWIN_writeRegiser(&data, DW_LAMBA_SURESI_ADR, sizeof(data));

				data = registerTable[DW_UST_ON_ISITICI_BANDI_ADR];
				DWIN_writeRegiser(&data, DW_UST_ON_ISITICI_BANDI_ADR, sizeof(data));

				data = registerTable[DW_UST_ARKA_ISITICI_BANDI_ADR];
				DWIN_writeRegiser(&data, DW_UST_ARKA_ISITICI_BANDI_ADR, sizeof(data));

				data = registerTable[DW_ALT_ISITICI_BANDI_ADR];
				DWIN_writeRegiser(&data, DW_ALT_ISITICI_BANDI_ADR, sizeof(data));

				data = registerTable[DW_ISITICI_UST_HIS_ADR];
				DWIN_writeRegiser(&data, DW_ISITICI_UST_HIS_ADR, sizeof(data));

				data = registerTable[DW_ISITICI_ALT_HIS_ADR];
				DWIN_writeRegiser(&data, DW_ISITICI_ALT_HIS_ADR, sizeof(data));

				data = registerTable[DW_ISITICI_PERIOD_ADR];
				DWIN_writeRegiser(&data, DW_ISITICI_PERIOD_ADR, sizeof(data));

				DWIN_changePage(86);
			}

			else
				DWIN_changePage(10);


		break;

		case DW_LAMBA_SURESI_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_LAMBA_SURESI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_LAMBA_SURESI_ADR, data2, sizeof(data2));

		break;

		case DW_UST_ON_ISITICI_BANDI_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_UST_ON_ISITICI_BANDI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_UST_ON_ISITICI_BANDI_ADR, data2, sizeof(data2));

		break;

		case DW_UST_ARKA_ISITICI_BANDI_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_UST_ARKA_ISITICI_BANDI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_UST_ARKA_ISITICI_BANDI_ADR, data2, sizeof(data2));

		break;

		case DW_ALT_ISITICI_BANDI_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_ALT_ISITICI_BANDI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_ALT_ISITICI_BANDI_ADR, data2, sizeof(data2));

		break;

		case DW_ISITICI_UST_HIS_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_ISITICI_UST_HIS_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_ISITICI_UST_HIS_ADR, data2, sizeof(data2));

		break;

		case DW_ISITICI_ALT_HIS_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_ISITICI_ALT_HIS_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_ISITICI_ALT_HIS_ADR, data2, sizeof(data2));

		break;

		case DW_ISITICI_PERIOD_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			registerTable[DW_ISITICI_PERIOD_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_ISITICI_PERIOD_ADR, data2, sizeof(data2));

			changeMaxSetValue(data);

		break;

		case DW_ARIZA_ALARM_SUSTURMA_ADR:

			data = 1;

			registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR] = 1;

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);
			setOut(K10, 0);

		break;

		case DW_TARIH_SAAT_PAGE_ENTER_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				uint8_t saniye, dakika, saat, gun, hafta ,ay, yil;

				DWIN_readRTC(&saniye, &dakika, &saat, &hafta, &gun, &ay, &yil);
				DWIN_writeRTC(saniye, dakika, saat, gun, ay, yil, 0);
			}

		break;


	}
}
void DWIN_manuelSayfa(void)
{

	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);

	switch(addr)
	{

		case DW_BUHAR_HAZIRLAMA_ADR:

			uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);
			registerTable[DW_BUHAR_HAZIRLAMA_ADR] = data;

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

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);
			registerTable[DW_BUHAR_PUSKURTME_ADR] = data;

			if(data == 0)
			{
				setOut(K9, 0);

				registerTable[DW_BUHAR_PUSKURTME_ADR] = 0;

				data = registerTable[DW_BUHAR_SURESI_ORT_ADR];
				registerTable[DW_BUHAR_SURESI_ADR] 	= registerTable[DW_BUHAR_SURESI_ORT_ADR];

				DWIN_writeRegiser(&data, DW_BUHAR_SURESI_ADR, sizeof(data));

			}

			else if((data == 1)&&(registerTable[DW_BUHAR_HAZIRLAMA_ADR] == 1))
			{
				setOut(K9, 1);

				registerTable[DW_BUHAR_PUSKURTME_ADR] = data;
				buharManuelDownCounter = registerTable[DW_BUHAR_SURESI_ADR];
				registerTable[DW_BUHAR_SURESI_ORT_ADR] = registerTable[DW_BUHAR_SURESI_ADR];
			}

			else if((data == 1)&&(registerTable[DW_BUHAR_HAZIRLAMA_ADR] == 0))
			{
				data = 0;
				registerTable[DW_BUHAR_PUSKURTME_ADR] = data;
				DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));
			}

		break;

		case DW_TURBO_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);
			registerTable[DW_TURBO_ADR] = data;

			if(data == 0)
			{
				altTurbo = 0;
				ustOnTurbo = 0;
				ustArkaTurbo = 0;
			}
			else
			{
				PID_Setup();
			}
			//setOut(K1|K3|K5|K6, data);

//			DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
//			DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
//			DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));
//			DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));
//			DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));

		break;

		case DW_UST_SICAKLIK_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				uint8_t data2[2];
				DWIN_readRegister(data2, DW_UST_SICAKLIK_SET_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);
				registerTable[DW_UST_SICAKLIK_SET_ADR] = data;

				DWIN_writeRegiser(&data, DW_UST_SICAKLIK_SET_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_UST_SICAKLIK_SET_ADR, data2, sizeof(data2));

				PID_Setup();
			}

		break;

		case DW_ALT_SICAKLIK_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				uint8_t data2[2];
				DWIN_readRegister(data2, DW_ALT_SICAKLIK_SET_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);
				registerTable[DW_ALT_SICAKLIK_SET_ADR] = data;

				DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_SET_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_ALT_SICAKLIK_SET_ADR, data2, sizeof(data2));

				PID_Setup();

			}

		break;

		case DW_UST_ON_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				uint8_t data2[2];
				DWIN_readRegister(data2, DW_UST_ON_SET_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);
				registerTable[DW_UST_ON_SET_ADR] = data;

				DWIN_writeRegiser(&data, DW_UST_ON_SET_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_UST_ON_SET_ADR, data2, sizeof(data2));
			}

		break;

		case DW_UST_ARKA_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				uint8_t data2[2];
				DWIN_readRegister(data2, DW_UST_ARKA_SET_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);
				registerTable[DW_UST_ARKA_SET_ADR] = data;

				DWIN_writeRegiser(&data, DW_UST_ARKA_SET_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_UST_ARKA_SET_ADR, data2, sizeof(data2));
			}

		break;

		case DW_ALT_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				uint8_t data2[2];
				DWIN_readRegister(data2, DW_ALT_SET_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);
				registerTable[DW_ALT_SET_ADR] = data;

				DWIN_writeRegiser(&data, DW_ALT_SET_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_ALT_SET_ADR, data2, sizeof(data2));
			}

		break;

		case DW_PISIRME_SURESI_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{

				uint8_t data2[2];
				DWIN_readRegister(data2, DW_PISIRME_SURESI_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);

				registerTable[DW_PISIRME_SURESI_ADR] 		= data;
				registerTable[DW_PISIRME_SURESI_ORT_ADR] 	= data;

				DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

				data = 0;

				DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_PISIRME_SURESI_ADR, data2, sizeof(data2));

			}

		break;

		case DW_BUHAR_SURESI_SET_ONAY_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{

				uint8_t data2[2];
				DWIN_readRegister(data2, DW_BUHAR_SURESI_ORT_ADR, sizeof(data2));

				data = combineBytes(data2[0], data2[1]);

				registerTable[DW_BUHAR_SURESI_ADR] 		= data;
				registerTable[DW_BUHAR_SURESI_ORT_ADR] 	= data;

				DWIN_writeRegiser(&data, DW_BUHAR_SURESI_ADR, sizeof(data));

				EEPROM_Write(&hi2c1, DW_BUHAR_SURESI_ADR, data2, sizeof(data2));
			}

		break;

		case DW_PISIRME_BASLATMA_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				if(registerTable[DW_PISIRME_BASLATMA_ADR] != 1)
				{
					registerTable[DW_PISIRME_BASLATMA_ADR] = data;
					pisirmeManuelDownCounter = (registerTable[DW_PISIRME_SURESI_ADR] * 60);
					registerTable[DW_PISIRME_SURESI_ORT_ADR] = registerTable[DW_PISIRME_SURESI_ADR];
				}
			}

		break;

		case DW_PISIRME_SONLANDIRMA_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

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

		break;

		case DW_PISIRME_ALARM_SUSTURMA_ADR:

			data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

			if(data == 1)
			{
				pisirmeSonuAlarmFlag = 0;
				pisirmeSonuAlarmBuzzer = 0;
				setOut(BUZZER, 0);
				setOut(K10, 0);

				registerTable[DW_PISIRME_BASLATMA_ADR] = 0;

				data = registerTable[DW_PISIRME_SURESI_ORT_ADR];
				registerTable[DW_PISIRME_SURESI_ADR] 	= registerTable[DW_PISIRME_SURESI_ORT_ADR];
				registerTable[DW_PISIRME_SURESI_SN_ADR] = 0;
				registerTable[MANUEL_SURE_SONU_ADR] 	= 0;

				DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

				data = 0;

				DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));
				DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
				DWIN_writeRegiser(&data, DW_PISIRME_SONLANDIRMA_ADR, sizeof(data));

				//DWIN_resetManuelPisirme();
			}

		break;

	}

}


void DWIN_receteSayfa(void)
{
	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);

	if((addr >= DW_RECETE_ILK_ADR)&&(addr <= DW_RECETE_SON_ADR))
	{

		islemdekiRecete = addr;

		uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

		uint16_t recete_num = addr - DW_RECETE_ILK_ADR + 1;

		uint16_t receteAdimPageNumList[4] = {DW_PISIRME_DUZEN_PAGE1_ADR,
											DW_PISIRME_DUZEN_PAGE2_ADR,
											DW_PISIRME_DUZEN_PAGE3_ADR,
											DW_PISIRME_DUZEN_PAGE4_ADR};

		uint8_t recete_all_data_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

		EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + ((recete_num - 1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_all_data_u8, sizeof(recete_all_data_u8));

		uint16_t receteResmi		=	combineBytes(recete_all_data_u8[76], recete_all_data_u8[77]);
		uint16_t receteAdimSayisi 	= 	combineBytes(recete_all_data_u8[78], recete_all_data_u8[79]);
		uint16_t receteAdi[DW_RECETE_ISIM_SIZE/2];

		for(int i=0;i<10;i++)
			receteAdi[i] = combineBytes(recete_all_data_u8[56+(i*2)], recete_all_data_u8[57+(i*2)]);

		DWIN_writeRegiser(&recete_num, DW_RECETE_DUZ_NUM_ADR, sizeof(recete_num));
		DWIN_writeRegiser(&receteResmi, DW_RECETE_DUZ_RESIM_ADR, sizeof(receteResmi));
		DWIN_writeRegiser(&receteAdimSayisi, DW_RECETE_DUZ_ADIM_SAY_ADR, sizeof(receteAdimSayisi));
		DWIN_writeRegiser(receteAdi, DW_RECETE_DUZ_ISIM_ADR, sizeof(receteAdi));

		switch(data)
		{

			case DW_RECETE_PISIRME_CMD:

				islemdekiReceteAdim = 1;

				registerTable[REG_DW_MODE_INFO_ADR] = DW_RECETE_PISIRME_SAYFA_ENTER;

				for(int i=0;i<((EE_RECETE_DATA_SIZE-4)/4)/2;i++)
				{
					uint16_t recete_pisirme_param = combineBytes(recete_all_data_u8[i*2], recete_all_data_u8[(i*2)+1]);
					DWIN_writeRegiser(&recete_pisirme_param, DW_RECETE_PISIR_UST_SIC_SET_ADR + (i*2), sizeof(recete_pisirme_param));
					registerTable[DW_RECETE_PISIR_UST_SIC_SET_ADR + (i*2)] = recete_pisirme_param;
				}



				uint16_t recete_pisirme_top_sure = 0;

				for(int i=0;i<receteAdimSayisi;i++)
					recete_pisirme_top_sure += combineBytes(recete_all_data_u8[(i*(EE_RECETE_DATA_SIZE-4)/4) + 10], recete_all_data_u8[(i*(EE_RECETE_DATA_SIZE-4)/4) + 11]);

				DWIN_writeRegiser(&recete_pisirme_top_sure, DW_RECETE_PISIR_SURE_ADR, sizeof(recete_pisirme_top_sure));

				registerTable[DW_PISIRME_SURESI_ADR] = recete_pisirme_top_sure;
				registerTable[DW_PISIRME_SURESI_ORT_ADR] 	= registerTable[DW_PISIRME_SURESI_ADR];
				registerTable[DW_BUHAR_SURESI_ORT_ADR] 		= registerTable[DW_BUHAR_SURESI_ADR];

				DWIN_changePage(DW_RECETE_PISIRME_PAGE_ADR);

				uint16_t adim_anim_list[4]= {DW_RECETE_A1_ANIM_ADR,
											DW_RECETE_A2_ANIM_ADR,
											DW_RECETE_A3_ANIM_ADR,
											DW_RECETE_A4_ANIM_ADR
				};

				uint16_t adim_anim_active_num[4]= {1,5,9,13};

				for(int i=0;i<receteAdimSayisi;i++)
				{
					uint16_t writeData = adim_anim_active_num[i];
					DWIN_writeRegiser(&writeData, adim_anim_list[i], sizeof(writeData));
				}

				for(int i=receteAdimSayisi;i<4;i++)
				{
					uint16_t writeData = 0x16;
					DWIN_writeRegiser(&writeData, adim_anim_list[i], sizeof(writeData));
				}

				setOut(K14, data);
				//DWIN_enterManuelProcess();
				PID_Setup();

			break;

			case DW_RECETE_DUZENLEME_CMD:

				registerTable[REG_DW_MODE_INFO_ADR] = DW_RECETE_DUZEN_SAYFA_ENTER;

				for(int i=0;i<28;i++)
				{
					uint16_t writeData = combineBytes(recete_all_data_u8[i*2], recete_all_data_u8[(i*2)+1]);
					DWIN_writeRegiser(&writeData, DW_RECETE_DUZ_UST_SIC_SET_ADR + (i*2), sizeof(writeData));
				}

				DWIN_changePage(receteAdimPageNumList[receteAdimSayisi - 1]);


			break;

		}
	}

	else if(addr == DW_RECETE_CIKIS_CMD)
	{
		uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

		switch(data)
		{
			case 1:

				registerTable[REG_DW_MODE_INFO_ADR] = DW_RECETE_SAYFA_ENTER;

				uint16_t recete_num = islemdekiRecete - DW_RECETE_ILK_ADR + 1;

				uint8_t Recete_data1_u8[EE_RECETE_DATA_SIZE - 2] 					= {0};
				uint8_t Recete_data2_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] 	= {0};
				uint8_t recete_adimSayisi[2] 										= {0};
				uint8_t recete_resim[2] 											= {0};
				uint16_t recete_resim_u16[1] 										= {0};
				uint8_t recete_isim[DW_RECETE_ISIM_SIZE] 							= {0};
				uint16_t recete_isim_u16[DW_RECETE_ISIM_SIZE/2] 					= {0};

				//////////////////////////////////////////////////////////////////////////////////////////////////////
				DWIN_readRegister(recete_isim, DW_RECETE_DUZ_ISIM_ADR, sizeof(recete_isim));

				uint8_t ff_check = 0;

				for(int i=0;i<DW_RECETE_ISIM_SIZE;i++)
				{
					if(ff_check != 1)
					{
						if(recete_isim[i] == 0xFF)
						{
							recete_isim[i] = 0;
							ff_check = 1;
						}
					}
					else
						recete_isim[i] = 0;
				}
				////////////////////////////////////////////////////////////////////////////////////////////////////
				for(int i=0;i<(EE_RECETE_DATA_SIZE/2)-2;i++)
				{
					uint8_t readData[2];
					DWIN_readRegister(readData, DW_RECETE_DUZ_UST_SIC_SET_ADR + (i*2), sizeof(readData));
					Recete_data1_u8[(i*2)] = readData[0];
					Recete_data1_u8[(i*2)+1] = readData[1];
				}
				//////////////////////////////////////////////////////////////////////////////////////////////////////


				DWIN_readRegister(recete_adimSayisi, DW_RECETE_DUZ_ADIM_SAY_ADR, sizeof(recete_adimSayisi));
				DWIN_readRegister(recete_resim, DW_RECETE_DUZ_RESIM_ADR, sizeof(recete_resim));


				for(int i=0;i<(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)-4;i++)
				{
					if(i<56)
						Recete_data2_u8[i] = Recete_data1_u8[i];
					else if(i<76)
						Recete_data2_u8[i] = recete_isim[i - 56];

				}

				Recete_data2_u8[(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)-4] = recete_resim[0];
				Recete_data2_u8[(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)-3] = recete_resim[1];

				Recete_data2_u8[(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)-2] = recete_adimSayisi[0];
				Recete_data2_u8[(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)-1] = recete_adimSayisi[1];

				convert_u8_to_u16(recete_isim, recete_isim_u16, sizeof(recete_isim));
				DWIN_writeRegiser(recete_isim_u16, DW_RECETE_ISIM_ILK_ADR + ((recete_num-1)*(DW_RECETE_ISIM_SIZE/2)), sizeof(recete_isim_u16));

				convert_u8_to_u16(recete_resim, recete_resim_u16, sizeof(recete_resim));
				DWIN_writeRegiser(recete_resim_u16, DW_RECETE_RESIM_ILK_ADR + (recete_num-1), sizeof(recete_resim_u16));

				EEPROM_Write(&hi2c1, EE_RECETE_ILK_ADR + ((recete_num - 1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), Recete_data2_u8, sizeof(Recete_data2_u8));



			break;

		}
		islemdekiRecete = 0;
	}

	else if((addr >= DW_UST_SICAKLIK_SET_ONAY_ADR)&&(addr <= DW_BUHAR_SURESI_SET_ONAY_ADR))
	{
		uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

		if(data == 1)
		{
			uint16_t ortak_adress[7] ={	DW_UST_SICAKLIK_SET_ORT_ADR,
										DW_ALT_SICAKLIK_SET_ORT_ADR,
										DW_UST_ON_SET_ORT_ADR,
										DW_UST_ARKA_SET_ORT_ADR,
										DW_ALT_SET_ORT_ADR,
										DW_PISIRME_SURESI_ORT_ADR,
										DW_BUHAR_SURESI_ORT_ADR
									};

			uint8_t readBytes[4];

			DWIN_readRegister(readBytes, ortak_adress[addr - DW_UST_SICAKLIK_SET_ONAY_ADR], sizeof(readBytes));
			HAL_Delay(0);

			uint16_t addres 		= combineBytes(readBytes[2], readBytes[3]);
			uint16_t data_recete	= combineBytes(readBytes[0], readBytes[1]);

			DWIN_writeRegiser(&data_recete, addres, sizeof(data_recete));
		}
	}
}

void DWIN_recetePisirmeAdimProcess(void)
{
	if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
	{
		uint16_t recete_eeprom_location = islemdekiRecete - DW_RECETE_ILK_ADR;

		uint8_t recete_all_data_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

		EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + (recete_eeprom_location *(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_all_data_u8, sizeof(recete_all_data_u8));

		uint16_t receteAdimSayisi = combineBytes(recete_all_data_u8[78], recete_all_data_u8[79]);

		uint16_t receteAdimSure[4] = {0};

		for(int i=0;i<receteAdimSayisi;i++)
			receteAdimSure[i] = 60 * combineBytes(recete_all_data_u8[(i*(EE_RECETE_DATA_SIZE-4)/4) + 10], recete_all_data_u8[(i*(EE_RECETE_DATA_SIZE-4)/4) + 11]);

		if(pisirmeManuelDownCounter > ((registerTable[DW_PISIRME_SURESI_ORT_ADR] * 60) - receteAdimSure[0]))
		{
			if(islemdekiReceteAdim != 1)
			{
				islemdekiReceteAdim = 1;
				SEGGER_RTT_printf(0,"A1 AKTIF !!!\r\n");

				uint16_t recete_num = islemdekiRecete - DW_RECETE_ILK_ADR + 1;

				uint8_t recete_all_data_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

				EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + ((recete_num - 1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_all_data_u8, sizeof(recete_all_data_u8));

				uint8_t islemdeki_recete_buffer_loc = (islemdekiReceteAdim - 1)*(((EE_RECETE_DATA_SIZE-4)/4)/2);

				uint16_t receteAdimSayisi 	= 	combineBytes(recete_all_data_u8[78], recete_all_data_u8[79]);

				for(int i= 0 + islemdeki_recete_buffer_loc; i<(((EE_RECETE_DATA_SIZE-4)/4)/2) + islemdeki_recete_buffer_loc; i++)
				{
					uint16_t recete_pisirme_param = combineBytes(recete_all_data_u8[i*2], recete_all_data_u8[(i*2)+1]);
					DWIN_writeRegiser(&recete_pisirme_param, DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2), sizeof(recete_pisirme_param));
					registerTable[DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2)] = recete_pisirme_param;
				}

				uint16_t adim_anim_list[4]= {DW_RECETE_A1_ANIM_ADR,
											DW_RECETE_A2_ANIM_ADR,
											DW_RECETE_A3_ANIM_ADR,
											DW_RECETE_A4_ANIM_ADR
				};

				uint16_t anim_bitirme = 0;
				uint16_t anim_baslatma = 1;

				uint16_t adim_anim_active_num[4]= {1,5,9,13};

				for(int i=0;i<receteAdimSayisi;i++)
				{
					uint16_t writeData = adim_anim_active_num[i];
					DWIN_writeRegiser(&writeData, adim_anim_list[i], sizeof(writeData));
				}

				for(int i=receteAdimSayisi;i<4;i++)
				{
					uint16_t writeData = 0x16;
					DWIN_writeRegiser(&writeData, adim_anim_list[i], sizeof(writeData));
				}

				for(int i=0;i<islemdekiReceteAdim;i++)
				{
					if((islemdekiReceteAdim - 1) == i)
						DWIN_writeRegiser(&anim_baslatma, adim_anim_list[i], sizeof(anim_baslatma));

					else
						DWIN_writeRegiser(&anim_bitirme, adim_anim_list[i], sizeof(anim_bitirme));
				}

				PID_Setup();
			}
		}

		else if(pisirmeManuelDownCounter > ((registerTable[DW_PISIRME_SURESI_ORT_ADR] * 60) - (receteAdimSure[0] + receteAdimSure[1])))
		{
			if(islemdekiReceteAdim != 2)
			{
				islemdekiReceteAdim = 2;
				SEGGER_RTT_printf(0,"A2 AKTIF !!!\r\n");

				uint16_t recete_num = islemdekiRecete - DW_RECETE_ILK_ADR + 1;

				uint8_t recete_all_data_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

				EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + ((recete_num - 1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_all_data_u8, sizeof(recete_all_data_u8));

				uint8_t islemdeki_recete_buffer_loc = (islemdekiReceteAdim - 1)*(((EE_RECETE_DATA_SIZE-4)/4)/2);

				//uint16_t receteAdimSayisi 	= 	combineBytes(recete_all_data_u8[78], recete_all_data_u8[79]);

				for(int i= 0 + islemdeki_recete_buffer_loc; i<(((EE_RECETE_DATA_SIZE-4)/4)/2) + islemdeki_recete_buffer_loc; i++)
				{
					uint16_t recete_pisirme_param = combineBytes(recete_all_data_u8[i*2], recete_all_data_u8[(i*2)+1]);
					DWIN_writeRegiser(&recete_pisirme_param, DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2), sizeof(recete_pisirme_param));
					registerTable[DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2)] = recete_pisirme_param;
				}

				uint16_t adim_anim_list[4]= {DW_RECETE_A1_ANIM_ADR,
											DW_RECETE_A2_ANIM_ADR,
											DW_RECETE_A3_ANIM_ADR,
											DW_RECETE_A4_ANIM_ADR
				};

				uint16_t anim_bitirme = 0;
				uint16_t anim_baslatma = 1;

				for(int i=0;i<islemdekiReceteAdim;i++)
				{
					if((islemdekiReceteAdim - 1) == i)
						DWIN_writeRegiser(&anim_baslatma, adim_anim_list[i], sizeof(anim_baslatma));

					else
						DWIN_writeRegiser(&anim_bitirme, adim_anim_list[i], sizeof(anim_bitirme));

				}

				PID_Setup();
			}
		}
		else if(pisirmeManuelDownCounter > ((registerTable[DW_PISIRME_SURESI_ORT_ADR] * 60) - (receteAdimSure[0] + receteAdimSure[1] + receteAdimSure[2])))
		{
			if(islemdekiReceteAdim != 3)
			{
				islemdekiReceteAdim = 3;
				SEGGER_RTT_printf(0,"A3 AKTIF !!!\r\n");

				uint16_t recete_num = islemdekiRecete - DW_RECETE_ILK_ADR + 1;

				uint8_t recete_all_data_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

				EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + ((recete_num - 1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_all_data_u8, sizeof(recete_all_data_u8));

				uint8_t islemdeki_recete_buffer_loc = (islemdekiReceteAdim - 1)*(((EE_RECETE_DATA_SIZE-4)/4)/2);

				//uint16_t receteAdimSayisi 	= 	combineBytes(recete_all_data_u8[78], recete_all_data_u8[79]);

				for(int i= 0 + islemdeki_recete_buffer_loc; i<(((EE_RECETE_DATA_SIZE-4)/4)/2) + islemdeki_recete_buffer_loc; i++)
				{
					uint16_t recete_pisirme_param = combineBytes(recete_all_data_u8[i*2], recete_all_data_u8[(i*2)+1]);
					DWIN_writeRegiser(&recete_pisirme_param, DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2), sizeof(recete_pisirme_param));
					registerTable[DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2)] = recete_pisirme_param;
				}

				uint16_t adim_anim_list[4]= {DW_RECETE_A1_ANIM_ADR,
											DW_RECETE_A2_ANIM_ADR,
											DW_RECETE_A3_ANIM_ADR,
											DW_RECETE_A4_ANIM_ADR
				};

				uint16_t anim_bitirme = 0;
				uint16_t anim_baslatma = 1;

				for(int i=0;i<islemdekiReceteAdim;i++)
				{
					if((islemdekiReceteAdim - 1) == i)
						DWIN_writeRegiser(&anim_baslatma, adim_anim_list[i], sizeof(anim_baslatma));

					else
						DWIN_writeRegiser(&anim_bitirme, adim_anim_list[i], sizeof(anim_bitirme));

				}

				PID_Setup();
			}
		}
		else if(pisirmeManuelDownCounter > ((registerTable[DW_PISIRME_SURESI_ORT_ADR] * 60) - (receteAdimSure[0] + receteAdimSure[1] + receteAdimSure[2] + receteAdimSure[3])))
		{
			if(islemdekiReceteAdim != 4)
			{
				islemdekiReceteAdim = 4;
				SEGGER_RTT_printf(0,"A4 AKTIF !!!\r\n");

				uint16_t recete_num = islemdekiRecete - DW_RECETE_ILK_ADR + 1;

				uint8_t recete_all_data_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE] = {0};

				EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + ((recete_num - 1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_all_data_u8, sizeof(recete_all_data_u8));

				uint8_t islemdeki_recete_buffer_loc = (islemdekiReceteAdim - 1)*(((EE_RECETE_DATA_SIZE-4)/4)/2);

				//uint16_t receteAdimSayisi 	= 	combineBytes(recete_all_data_u8[78], recete_all_data_u8[79]);

				for(int i= 0 + islemdeki_recete_buffer_loc; i<(((EE_RECETE_DATA_SIZE-4)/4)/2) + islemdeki_recete_buffer_loc; i++)
				{
					uint16_t recete_pisirme_param = combineBytes(recete_all_data_u8[i*2], recete_all_data_u8[(i*2)+1]);
					DWIN_writeRegiser(&recete_pisirme_param, DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2), sizeof(recete_pisirme_param));
					registerTable[DW_RECETE_PISIR_UST_SIC_SET_ADR + ((i - islemdeki_recete_buffer_loc)*2)] = recete_pisirme_param;
				}

				uint16_t adim_anim_list[4]= {DW_RECETE_A1_ANIM_ADR,
											DW_RECETE_A2_ANIM_ADR,
											DW_RECETE_A3_ANIM_ADR,
											DW_RECETE_A4_ANIM_ADR
				};

				uint16_t anim_bitirme = 0;
				uint16_t anim_baslatma = 1;

				for(int i=0;i<islemdekiReceteAdim;i++)
				{
					if((islemdekiReceteAdim - 1) == i)
						DWIN_writeRegiser(&anim_baslatma, adim_anim_list[i], sizeof(anim_baslatma));

					else
						DWIN_writeRegiser(&anim_bitirme, adim_anim_list[i], sizeof(anim_bitirme));

				}

				PID_Setup();
			}
		}

	}
}

void DWIN_resetManuelPisirme(void)
{
	pisirmeSonuAlarmFlag = 0;
	pisirmeSonuAlarmBuzzer = 0;

	altSicaklikProcess = 99;
	ustSicaklikProcess = 99;

	altTurbo 		= 0;
	ustArkaTurbo 	= 0;
	ustOnTurbo 		= 0;
	alarmBuzzerPeriod = 1000;

	setOut(K8|K9|BUZZER|K1|K2|K3|K4|K5|K6|K14|K10, 0);


	uint16_t data;

/////////////////////////////////////////////////////////////////////////////////////////////////////

	registerTable[DW_PISIRME_SURESI_ADR] 	= registerTable[DW_PISIRME_SURESI_ORT_ADR];
	registerTable[DW_PISIRME_SURESI_SN_ADR] = 0;
	data = registerTable[DW_PISIRME_SURESI_ORT_ADR];
	DWIN_writeRegiser(&data, DW_PISIRME_SURESI_ADR, sizeof(data));

////////////////////////////////////////////////////////////////////////////////////////////////////

	registerTable[DW_BUHAR_SURESI_ADR] 	= registerTable[DW_BUHAR_SURESI_ORT_ADR];
	registerTable[DW_BUHAR_PUSKURTME_ADR] = 0;
	data = registerTable[DW_BUHAR_SURESI_ORT_ADR];
	DWIN_writeRegiser(&data, DW_BUHAR_SURESI_ADR, sizeof(data));

////////////////////////////////////////////////////////////////////////////////////////////////////

	registerTable[DW_PISIRME_BASLATMA_ADR] 		= 0;
	registerTable[DW_BUHAR_HAZIRLAMA_ADR] 		= 0;
	registerTable[DW_BUHAR_PUSKURTME_ADR] 		= 0;
	registerTable[DW_TURBO_ADR] 				= 0;
	registerTable[DW_UST_SICAKLIK_ANIM] 		= 0;
	registerTable[DW_UST_ON_ANIM] 				= 0;
	registerTable[DW_UST_ARKA_ANIM] 			= 0;
	registerTable[DW_ALT_SICAKLIK_ANIM] 		= 0;
	registerTable[DW_ALT_ANIM] 					= 0;
	registerTable[DW_ARIZA_PAGE_ADR]			= 0;
	registerTable[MANUEL_SURE_SONU_ADR] 		= 0;
	registerTable[DW_BUHAR_HAZIR_ANIM]			= 0;
	registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR]	= 0;

	data = 0;

	DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_PISIRME_SONLANDIRMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_BUHAR_HAZIRLAMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_UST_ON_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_UST_ARKA_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_ALT_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_ARIZA_ALARM_SUSTURMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_BUHAR_HAZIR_ANIM, sizeof(data));
}

void DWIN_arızaCheck(void)
{
	if((temp.TC2 >= 400) && (registerTable[DW_TC2_ARIZA_ADR] != 1))
	{
		registerTable[DW_TC2_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_TC2_ARIZA_ADR, sizeof(data));
	}

	if((temp.TC2 < 400) && (registerTable[DW_TC2_ARIZA_ADR] != 0))
	{
		registerTable[DW_TC2_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_TC2_ARIZA_ADR, sizeof(data));
	}

	if((temp.TC3 >= 400) && (registerTable[DW_TC3_ARIZA_ADR] != 1))
	{
		registerTable[DW_TC3_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_TC3_ARIZA_ADR, sizeof(data));
	}

	if((temp.TC3 < 400) && (registerTable[DW_TC3_ARIZA_ADR] != 0))
	{
		registerTable[DW_TC3_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_TC3_ARIZA_ADR, sizeof(data));
	}

	if((HAL_GPIO_ReadPin(I_KAPI_SWITCH) == 0)&&(registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] != 1))
	{
		registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_ASIRI_SICAKLIK_ARIZA_ADR, sizeof(data));
	}

	if((HAL_GPIO_ReadPin(I_KAPI_SWITCH) == 1)&&(registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] != 0))
	{
		registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_ASIRI_SICAKLIK_ARIZA_ADR, sizeof(data));
	}

	if(((registerTable[DW_TC2_ARIZA_ADR] == 1) || (registerTable[DW_TC3_ARIZA_ADR] == 1) || (registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] == 1))&&(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		registerTable[DW_ARIZA_PAGE_ADR] = 1;
		DWIN_changePage(DW_ARIZA_PAGE_ADR);
		setOut(K1|K2|K3|K4|K5|K6, 0);
		setOut(K10, 1);
		pisirmeSonuAlarmFlag = 1;
		alarmBuzzerPeriod = 200;
	}

	if((registerTable[DW_TC2_ARIZA_ADR] == 0) && (registerTable[DW_TC3_ARIZA_ADR] == 0) && (registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] == 0) && (registerTable[DW_ARIZA_PAGE_ADR] != 0))
	{
		registerTable[DW_ARIZA_PAGE_ADR] = 0;
		registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_ARIZA_ALARM_SUSTURMA_ADR, sizeof(data));

		if(registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)
			DWIN_changePage(DW_PISIRME_PAGE_ADR);
		else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
			DWIN_changePage(DW_RECETE_PISIRME_PAGE_ADR);


		if(registerTable[MANUEL_SURE_SONU_ADR] != 1)
		{

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);
			setOut(K10, 0);
		}
		alarmBuzzerPeriod = 1000;

		if(registerTable[MANUEL_SURE_SONU_ADR] == 1)
		{
			if(registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)
				DWIN_changePage(MANUEL_SURE_SONU_ADR);
			else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
				DWIN_changePage(RECETE_SURE_SONU_ADR);
		}


		ustSicaklikProcess = 99;
		altSicaklikProcess = 99;

	}
}

void DWIN_buharHazirCheck(void)
{
	if(HAL_GPIO_ReadPin(I_BUHAR_HAZIR) == 1)
		counterTick.buharHazir = 0;

	if((counterTick.buharHazir >= 100) && (registerTable[DW_BUHAR_HAZIRLAMA_ADR] == 1) && (registerTable[DW_BUHAR_HAZIR_ANIM] != 2))
	{
		registerTable[DW_BUHAR_HAZIR_ANIM] = 2;
		uint16_t data = 2;
		DWIN_writeRegiser(&data, DW_BUHAR_HAZIR_ANIM, sizeof(data));
	}

	if((counterTick.buharHazir < 100) && (registerTable[DW_BUHAR_HAZIRLAMA_ADR] == 1) && (registerTable[DW_BUHAR_HAZIR_ANIM] == 2))
	{
		registerTable[DW_BUHAR_HAZIR_ANIM] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_BUHAR_HAZIR_ANIM, sizeof(data));
	}

}

void DWIN_writeRTC(uint8_t saniye, uint8_t dakika, uint8_t saat, uint8_t gun, uint8_t ay, uint8_t yil, uint8_t writeEN)
{
	uint16_t rtcEnable_msg = DW_WRITE_RTC_DONE_MSG;

	uint16_t dk_sn 		= combineBytes(dakika, saniye);
	uint16_t gun_saat 	= combineBytes(gun, saat);
	uint16_t yil_ay		= combineBytes(yil, ay);

	if(writeEN)
	{
		uint16_t writeBuffer[4] = {rtcEnable_msg, yil_ay, gun_saat, dk_sn};
		DWIN_writeRegiser(writeBuffer, DW_FIRST_WRITE_RTC_ADR, sizeof(writeBuffer));
	}
	else
	{
		uint16_t writeBuffer[3] = {yil_ay, gun_saat, dk_sn};
		DWIN_writeRegiser(writeBuffer, DW_FIRST_WRITE_RTC_ADR + 1, sizeof(writeBuffer));
	}

}

void DWIN_readRTC(uint8_t* saniye, uint8_t* dakika, uint8_t* saat, uint8_t* hafta, uint8_t* gun, uint8_t* ay, uint8_t* yil)
{
	uint8_t readBuffer[8] = {0};

	DWIN_readRegister(readBuffer, DW_FIRST_READ_RTC_ADR, sizeof(readBuffer));

	*yil 	= readBuffer[0];
	*ay	 	= readBuffer[1];
	*gun 	= readBuffer[2];
	*hafta	= readBuffer[3];
	*saat 	= readBuffer[4];
	*dakika = readBuffer[5];
	*saniye = readBuffer[6];
}

