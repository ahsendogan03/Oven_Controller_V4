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
#include "Bluetooth_Process.h"
#include "tim.h"
#include "dac.h"

extern TemperatureData temp;
extern uint8_t DWIN_rxBuffer[DWIN_rxBufferSize];
extern uint8_t main_DWIN_rxBuffer[DWIN_rxBufferSize];

extern USART_TypeDef *DWIN_usartDeclaration;
extern UART_HandleTypeDef *DWIN_huart_channel;
extern DMA_HandleTypeDef *DWIN_hdma_usart_purpose;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern I2C_HandleTypeDef hi2c1;

extern usartInfo DWIN;

extern uint16_t eepromAddrTable[EEPROM_TABLE_LEN];

extern uint32_t outputData;

tickCounter counterTick;

uint16_t registerTable[REGISTER_TABLE_SIZE];


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

uint16_t islemdekiRecete 			= 0;
uint16_t islemdekiReceteAdim 		= 1;
uint16_t islemdekiOtomatikGun		= 0;
uint16_t islemdekiOtomatikAktifIkon	= 0;

uint8_t otomatikPisirmeBaslatmaFlag = 0;

volatile uint32_t start_freq = 0;      // Geçiş başladığındaki frekans
volatile uint32_t target_freq = 0;     // Hedeflenen frekans
volatile uint32_t current_freq = 0;    // Anlık hesaplanan frekans
volatile uint16_t tick_counter = 0;    // 0'dan 1000'e kadar sayacak
volatile uint8_t  current_duty = 0;    // Sabit kalacak duty cycle
volatile uint8_t  is_transitioning = 0; // Geçiş devam ediyor mu?

static inline void UserApp_IWDG_Refresh(void)
{
    IWDG->KR = 0xAAAA;
}

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
        dest[2 * i + 1]     = (uint8_t)(src[i] & 0xFF);       	// Lower byte
        dest[2 * i] 		= (uint8_t)((src[i] >> 8) & 0xFF);  // Higher byte
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
	DWIN_otomatikAcmaCheck();
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

void dwin_popup_change_structure(uint16_t page, uint8_t controlID, uint16_t picNext)
{
	uint16_t tx[4];


	tx[0]=(uint16_t)(0x5A<<0x08)+(uint16_t)0xA5;
	tx[1]=page;
	tx[2]=(uint16_t)(controlID<<0x08)+0x01;
	tx[3]=0x0002;

	DWIN_writeRegiser(tx, 0x00b0, sizeof(tx));

	HAL_Delay(50);

	uint8_t readData[48] = {0};
	DWIN_readRegister(readData, 0x00B4, sizeof(readData));

	uint16_t data[28];

	data[0]=(uint16_t)(0x5A<<0x08)+(uint16_t)0xA5;
	data[1]=page;
	data[2]=(uint16_t)(controlID<<0x08)+0x01;
	data[3]=0x0003;

	for(int i=0;i<sizeof(readData)/2;i++)
		data[i+4] = combineBytes(readData[i*2], readData[(i*2)+1]);

    data[14]=picNext;

    DWIN_writeRegiser(data, 0x00b0, sizeof(data));

    HAL_Delay(50);


}

void dwin_pop_up_change_all_structure(uint16_t popPage)
{
	for(int i=12; i<29; i++)
	{
		for(int j=0; j<6; j++)
		{
			if(i != 28)
				dwin_popup_change_structure(i,(uint8_t)j,popPage);
			else if(j<4)
			{
				dwin_popup_change_structure(i,(uint8_t)j,popPage);
			}

		}

	}
}

void DWIN_changePopup(uint16_t dil)
{

	switch(dil)
	{
		case DW_DIL_TURKCE_VAL:
			dwin_pop_up_change_all_structure(11);
		break;
		case DW_DIL_INGILIZCE_VAL:
			dwin_pop_up_change_all_structure(5);
		break;
		case DW_DIL_RUSCA_VAL:
			dwin_pop_up_change_all_structure(6);
		break;
		case DW_DIL_ALMANCA_VAL:
			dwin_pop_up_change_all_structure(104);
		break;
	}
}

void dwin_icon_change_structure(uint16_t sp,uint16_t iconMin, uint16_t iconMax, uint16_t file)
{
	uint16_t writeData[3] = {iconMin, iconMax, file};
	DWIN_writeRegiser(writeData, sp+0x05, sizeof(writeData));
}

void DWIN_changeIcon(uint16_t dil)
{
	switch(dil)
	{
		case DW_DIL_TURKCE_VAL:

			dwin_icon_change_structure( 0x4700, 0x0028, 0x0027, 0x3701);

			dwin_icon_change_structure( 0x470A, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x4714, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x471E, 0x0028, 0x0027, 0x3701);

			dwin_icon_change_structure( 0x4728, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x4732, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x473C, 0x0000, 0x0000, 0x1E01);

			dwin_icon_change_structure( 0x4746, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x4750, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x475A, 0x0000, 0x0000, 0x1E01);

			dwin_icon_change_structure( 0x4764, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x476E, 0x0000, 0x0000, 0x1E01);

			dwin_icon_change_structure( 0x4778, 0x0028, 0x0027, 0x3701);

			dwin_icon_change_structure( 0x4782, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x478C, 0x0000, 0x0000, 0x1E01);

			dwin_icon_change_structure( 0x4796, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x47A0, 0x0028, 0x0027, 0x3701);

			dwin_icon_change_structure( 0x47AA, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x47B4, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x47BE, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x47C8, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure( 0x47D2, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure( 0x47DC, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure( 0x47E6, 0x002d, 0x002e, 0x3701);

			dwin_icon_change_structure( 0x47F0, 0x0030, 0x002f, 0x3701);

			dwin_icon_change_structure( 0x47FA, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure( 0x4804, 0x000B, 0x0011, 0X3700);

			dwin_icon_change_structure( 0x480E, 0x0012, 0x0013, 0x3700);

			dwin_icon_change_structure( 0x4818, 0x0012, 0x0013, 0x3700);

			dwin_icon_change_structure( 0x4822, 0x0004, 0x0005, 0x3700);

			dwin_icon_change_structure( 0x482C, 0x0004, 0x0006, 0X3700);

			dwin_icon_change_structure( 0x4836, 0x0004, 0x0007, 0x3700);

			dwin_icon_change_structure( 0x4840, 0x0004, 0x0008, 0x3700);

			dwin_icon_change_structure( 0x484A, 0x0004, 0x0009, 0x3700);

			dwin_icon_change_structure( 0x4854, 0x0004, 0x0025, 0x3700);

			dwin_icon_change_structure( 0x485E, 0x000a, 0x000a, 0x3700);



			if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 1)
			{
				dwin_icon_change_structure( 0x4868, 0x0016, 0x0016, 0x3700);

				dwin_icon_change_structure( 0x4872, 0x0016, 0x0016, 0X3700);

				dwin_icon_change_structure( 0x487C, 0x0016, 0x0016, 0x3700);

				dwin_icon_change_structure( 0x4886, 0x0016, 0x0016, 0x3700);

				dwin_icon_change_structure( 0x4890, 0x0016, 0x0016, 0x3700);

				dwin_icon_change_structure( 0x489A, 0x0016, 0x0016, 0x3700);

				dwin_icon_change_structure( 0x48A4, 0x0016, 0x0016, 0x3700);
			}
			else
			{
				dwin_icon_change_structure(0x4868, 0x009F,0x009F, 0x3A01);

				dwin_icon_change_structure(0x4872, 0x009F,0x009F, 0x3A01);

				dwin_icon_change_structure(0x487C, 0x009F,0x009F, 0x3A01);

				dwin_icon_change_structure(0x4886, 0x009F,0x009F, 0x3A01);

				dwin_icon_change_structure(0x4890, 0x009F,0x009F, 0x3A01);

				dwin_icon_change_structure(0x489A, 0x009F,0x009F, 0x3A01);

				dwin_icon_change_structure(0x48A4, 0x009F,0x009F, 0x3A01);

			}

			dwin_icon_change_structure( 0x48AE, 0x003a, 0x003a, 0x3701);

			dwin_icon_change_structure(0x5000,0x009A,0x009A,0x3A01);

			dwin_icon_change_structure(0x500A, 0x0094, 0x0094, 0x3A01);

			dwin_icon_change_structure(0x5014, 0x0097, 0x0097, 0x3A01);

			dwin_icon_change_structure( 0x501E, 0x003D, 0x003D, 0x3701);

			dwin_icon_change_structure( 0x5028, 0x0040, 0x0040, 0x3701);


		break;

		case DW_DIL_INGILIZCE_VAL:

			dwin_icon_change_structure(0x4700, 0x0061, 0x0069, 0x3A01);

			dwin_icon_change_structure(0x470A, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x4714, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x471E, 0x0061, 0x0069, 0x3A01);

			dwin_icon_change_structure(0x4728, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x4732, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x473C, 0x005E, 0x005E, 0x3A01);

			dwin_icon_change_structure(0x4746, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x4750, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x475A, 0x005E, 0x005E, 0x3A01);

			dwin_icon_change_structure(0x4764, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x476E, 0x005E, 0x005E, 0x3A01);

			dwin_icon_change_structure(0x4778, 0x0061, 0x0069, 0x3A01);

			dwin_icon_change_structure(0x4782, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x478C, 0x005E, 0x005E, 0x3A01);

			dwin_icon_change_structure(0x4796, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x47A0, 0x0061, 0x0069, 0x3A01);

			dwin_icon_change_structure(0x47AA, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x47B4, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x47BE, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure(0x47C8, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure(0x47D2, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure(0x47DC, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure(0x47E6, 0x0063, 0x0065, 0x3a01);

			dwin_icon_change_structure(0x47F0, 0x0067, 0x008f, 0x3a01);

			dwin_icon_change_structure(0x47FA, 0x0012, 0x0013, 0x3701);

			dwin_icon_change_structure(0x4804, 0X007B, 0X0081, 0x3A01);

			dwin_icon_change_structure(0x480E, 0x0012, 0x0013, 0x3700);

			dwin_icon_change_structure(0x4818, 0x0012, 0x0013, 0x3700);

			dwin_icon_change_structure(0x4822, 0x0091, 0x006e, 0x3A00);

			dwin_icon_change_structure(0x482C, 0x0091, 0x006f, 0x3A00);

			dwin_icon_change_structure(0x4836, 0x0091, 0x0070, 0x3A00);

			dwin_icon_change_structure(0x4840, 0x0091, 0x0071, 0x3A00);

			dwin_icon_change_structure(0x484A, 0x0091, 0x0072, 0x3A00);

			dwin_icon_change_structure(0x4854, 0x0091, 0x0073, 0x3A00);

			dwin_icon_change_structure(0x485E, 0x006c, 0x006c, 0x3A00);

			if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 1)
			{
				dwin_icon_change_structure(0x4868, 0x0089, 0x0089, 0x3A00);

				dwin_icon_change_structure(0x4872, 0x0089, 0x0089, 0x3A00);

				dwin_icon_change_structure(0x487C, 0x0089, 0x0089, 0x3A00);

				dwin_icon_change_structure(0x4886, 0x0089, 0x0089, 0x3A00);

				dwin_icon_change_structure(0x4890, 0x0089, 0x0089, 0x3A00);

				dwin_icon_change_structure(0x489A, 0x0089, 0x0089, 0x3A00);

				dwin_icon_change_structure(0x48A4, 0x0089, 0x0089, 0x3A00);

			}
			else
			{
					dwin_icon_change_structure(0x4868, 0x009D,0x009D, 0x3A01);

					dwin_icon_change_structure(0x4872, 0x009D,0x009D, 0x3A01);

					dwin_icon_change_structure(0x487C, 0x009D,0x009D, 0x3A01);

					dwin_icon_change_structure(0x4886, 0x009D,0x009D, 0x3A01);

					dwin_icon_change_structure(0x4890, 0x009D,0x009D, 0x3A01);

					dwin_icon_change_structure(0x489A, 0x009D,0x009D, 0x3A01);

					dwin_icon_change_structure(0x48A4, 0x009D,0x009D, 0x3A01);
			}



			dwin_icon_change_structure(0x48AE, 0x003b, 0x003b, 0x3701);


			dwin_icon_change_structure(0x5000, 0x009B, 0x009B, 0x3A01);

			dwin_icon_change_structure(0x500A, 0x0095, 0x0095, 0x3A01);

			dwin_icon_change_structure(0x5014, 0x0098, 0x0098, 0x3A01);

			dwin_icon_change_structure( 0x501E, 0x003E, 0x003E, 0x3701);

			dwin_icon_change_structure( 0x5028, 0x0041, 0x0041, 0x3701);



		break;

		case DW_DIL_RUSCA_VAL:

			dwin_icon_change_structure( 0x4700, 0x0062, 0x0060, 0x3A01);

			dwin_icon_change_structure( 0x470A, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x4714, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x471E, 0x0062, 0x0060, 0x3A01);

			dwin_icon_change_structure( 0x4728, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x4732, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x473C, 0x005F, 0x005F, 0x3A01);

			dwin_icon_change_structure( 0x4746, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x4750, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x475A, 0x005F, 0x005F, 0x3A01);

			dwin_icon_change_structure( 0x4764, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x476E, 0x005F, 0x005F, 0x3A01);

			dwin_icon_change_structure( 0x4778, 0x0062, 0x0060, 0x3A01);

			dwin_icon_change_structure( 0x4782, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x478C, 0x005F, 0x005F, 0x3A01);

			dwin_icon_change_structure( 0x4796, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x47A0, 0x0062, 0x0060, 0x3A01);

			dwin_icon_change_structure( 0x47AA, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x47B4, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x47BE, 0x00C8, 0x007A, 0x3A01);

			dwin_icon_change_structure( 0x47C8, 0x006a, 0x006b, 0x3A01);

			dwin_icon_change_structure( 0x47D2, 0x006a, 0x006b, 0x3A01);

			dwin_icon_change_structure( 0x47DC, 0x006a, 0x006b, 0x3A01);

			dwin_icon_change_structure( 0x47E6, 0x0064, 0x0066, 0x3A01);

			dwin_icon_change_structure( 0x47F0, 0x0068, 0x0090, 0x3a01);

			dwin_icon_change_structure( 0x47FA, 0x006a, 0x006b, 0x3A01);

			dwin_icon_change_structure( 0x4804, 0X0082, 0X0088, 0x3A01);

			dwin_icon_change_structure( 0x480E, 0x006b, 0x006a, 0x3A00);

			dwin_icon_change_structure( 0x4818, 0x006b, 0x006a, 0x3A00);

			dwin_icon_change_structure( 0x4822, 0x0091, 0x0074, 0x3A00);

			dwin_icon_change_structure( 0x482C, 0x0091, 0x0075, 0x3A00);

			dwin_icon_change_structure( 0x4836, 0x0091, 0x0076, 0x3A00);

			dwin_icon_change_structure( 0x4840, 0x0091, 0x0077, 0x3A00);

			dwin_icon_change_structure( 0x484A, 0x0091, 0x0078, 0x3A00);

			dwin_icon_change_structure( 0x4854, 0x0091, 0x0079, 0x3A00);

			dwin_icon_change_structure( 0x485E, 0x006d, 0x006d, 0x3A00);

			if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 1) //if dual temperature control is active, change the icons to match the dual temperature control mode
			{
				dwin_icon_change_structure( 0x4868, 0x008a, 0x008a, 0x3A00);

				dwin_icon_change_structure( 0x4872, 0x008a, 0x008a, 0x3A00);

				dwin_icon_change_structure( 0x487C, 0x008a, 0x008a, 0x3A00);

				dwin_icon_change_structure( 0x4886, 0x008a, 0x008a, 0x3A00);

				dwin_icon_change_structure( 0x4890, 0x008a, 0x008a, 0x3A00);

				dwin_icon_change_structure( 0x489A, 0x008a, 0x008a, 0x3A00);

				dwin_icon_change_structure( 0x48A4, 0x008a, 0x008a, 0x3A00);
			}
			else
			{
				dwin_icon_change_structure(0x4868, 0x009e,0x009e, 0x3A01);

				dwin_icon_change_structure(0x4872, 0x009e,0x009e, 0x3A01);

				dwin_icon_change_structure(0x487C, 0x009e,0x009e, 0x3A01);

				dwin_icon_change_structure(0x4886, 0x009e,0x009e, 0x3A01);

				dwin_icon_change_structure(0x4890, 0x009e,0x009e, 0x3A01);

				dwin_icon_change_structure(0x489A, 0x009e,0x009e, 0x3A01);

				dwin_icon_change_structure(0x48A4, 0x009e,0x009e, 0x3A01);
			}



			dwin_icon_change_structure( 0x48AE, 0x003c, 0x003c, 0x3701);

			dwin_icon_change_structure(0x5000, 0x009C, 0x009C, 0x3A01);

			dwin_icon_change_structure(0x500A, 0x0096, 0x0096, 0x3A01);

			dwin_icon_change_structure(0x5014, 0x0099, 0x0099, 0x3A01);

			dwin_icon_change_structure( 0x501E, 0x003F, 0x003F, 0x3701);

			dwin_icon_change_structure( 0x5028, 0x0042, 0x0042, 0x3701);



		break;

		case DW_DIL_ALMANCA_VAL:

			dwin_icon_change_structure( 0x4700, 0x002E, 0x002D, 0x3E01);

			dwin_icon_change_structure( 0x470A, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x4714, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x471E, 0x002E, 0x002D, 0x3E01);

			dwin_icon_change_structure( 0x4728, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x4732, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x473C, 0x002C, 0x002C, 0x3E01);

			dwin_icon_change_structure( 0x4746, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x4750, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x475A, 0x002C, 0x002C, 0x3E01);

			dwin_icon_change_structure( 0x4764, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x476E, 0x002C, 0x002C, 0x3E01);

			dwin_icon_change_structure( 0x4778, 0x002E, 0x002D, 0x3E01);

			dwin_icon_change_structure( 0x4782, 0x0000, 0x0001, 0x3701);

			dwin_icon_change_structure( 0x478C, 0x002C, 0x002C, 0x3E01);

			dwin_icon_change_structure( 0x47C8, 0x0033, 0x0034, 0x3E00);

			dwin_icon_change_structure( 0x47D2, 0x0033, 0x0034, 0x3E00);

			dwin_icon_change_structure( 0x47DC, 0x0033, 0x0034, 0x3E00);

			dwin_icon_change_structure( 0x47E6, 0x002F, 0x0030, 0x3E01);

			dwin_icon_change_structure( 0x47F0, 0x0031, 0x0032, 0x3E01);

			dwin_icon_change_structure( 0x47FA, 0x0033, 0x0034, 0x3E00);

			dwin_icon_change_structure( 0x4796, 0x0000, 0x0001, 0x3701);


			dwin_icon_change_structure( 0x47A0, 0x002E, 0x002D, 0x3E01);

			dwin_icon_change_structure( 0x4822, 0x0046, 0x0036, 0x3E01);

			dwin_icon_change_structure( 0x482C, 0x0046, 0x0037, 0x3E01);

			dwin_icon_change_structure( 0x4836, 0x0046, 0x0038, 0x3E01);

			dwin_icon_change_structure( 0x4840, 0x0046, 0x0039, 0x3E01);

			dwin_icon_change_structure( 0x484A, 0x0046, 0x003B, 0x3E01);

			dwin_icon_change_structure( 0x4854, 0x0046, 0x003A, 0x3E01);

			dwin_icon_change_structure( 0x485E, 0x0035, 0x0035, 0x3E01);

			if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 1) //if dual temperature control is active, change the icons to match the dual temperature control mode
			{
				dwin_icon_change_structure( 0x4868, 0x004B, 0x004B, 0x3E01);

				dwin_icon_change_structure( 0x4872, 0x004B, 0x004B, 0x3E01);

				dwin_icon_change_structure( 0x487C, 0x004B, 0x004B, 0x3E01);

				dwin_icon_change_structure( 0x4886, 0x004B, 0x004B, 0x3E01);

				dwin_icon_change_structure( 0x4890, 0x004B, 0x004B, 0x3E01);

				dwin_icon_change_structure( 0x489A, 0x004B, 0x004B, 0x3E01);

				dwin_icon_change_structure( 0x48A4, 0x004B, 0x004B, 0x3E01);




			}
			else
			{
				dwin_icon_change_structure( 0x4868, 0x0043, 0x0043, 0x3E01);
				dwin_icon_change_structure( 0x4872, 0x0043, 0x0043, 0x3E01);
				dwin_icon_change_structure( 0x487C, 0x0043, 0x0043, 0x3E01);
				dwin_icon_change_structure( 0x4886, 0x0043, 0x0043, 0x3E01);
				dwin_icon_change_structure( 0x4890, 0x0043, 0x0043, 0x3E01);
				dwin_icon_change_structure( 0x489A, 0x0043, 0x0043, 0x3E01);
				dwin_icon_change_structure( 0x48A4, 0x0043, 0x0043, 0x3E01);

			}

			dwin_icon_change_structure( 0x48AE, 0x0047, 0x0047, 0x3E01);

			dwin_icon_change_structure( 0x5000, 0x004A, 0x004A, 0x3E01);

			dwin_icon_change_structure( 0x4804, 0x003C, 0x0042, 0x3E01);

			dwin_icon_change_structure( 0x480E, 0x0033, 0x0034, 0x3E00);

			dwin_icon_change_structure( 0x4818, 0x0033, 0x0034, 0x3E00);

			dwin_icon_change_structure(0x500A, 0x0095, 0x0095, 0x3A01);

			dwin_icon_change_structure(0x5014, 0x0098, 0x0098, 0x3A01);

			dwin_icon_change_structure( 0x501E, 0x004F, 0x004F, 0x3E01);

			dwin_icon_change_structure( 0x5028, 0x0043, 0x0043, 0x3701);


		break;
	}
}



void keyboard_change_structure(uint16_t page,uint8_t touchID, uint8_t libID, uint8_t xDots, uint8_t yDots, uint8_t picture)
{
	uint16_t tx[4];

	tx[0]=0x5AA5;
	tx[1]=page;
	tx[2]=(uint16_t)(touchID<<0x08)+0x06;
	tx[3]=0x0002;

	DWIN_writeRegiser(tx, 0x00b0, sizeof(tx));
	HAL_Delay(50);

	uint8_t readData[64] = {0};
	DWIN_readRegister(readData, 0x00B4, sizeof(readData));


	uint16_t data[36];

	data[0]=0x5AA5;
	data[1]=page;
	data[2]=(uint16_t)(touchID<<0x08)+0x06;
	data[3]=0x0003;

	for(int i=0;i<sizeof(readData)/2;i++)
		data[i+4] = combineBytes(readData[i*2], readData[(i*2)+1]);

    data[0xa+0x04]=(data[0x0a+0x04] & 0xff00) | (uint16_t)(libID);
    data[0xb+0x04]=(uint16_t)(xDots<<0x08) | (uint16_t)(yDots);
    data[0x13+0x04]=(data[0x13+0x04] & 0xFf00) | (uint16_t)(picture);


    DWIN_writeRegiser(data, 0x00b0, sizeof(data));

    HAL_Delay(50);

}

void returnkeycode_change_structure(uint16_t picNext) //bu fonksiyon sadece tek bir tuş için geçerlidir
										//genel bir fonksiyon değildir
{
	uint16_t tx[4];

	tx[0]=0x5AA5;
	tx[1]=0000;
	tx[2]=0x0005;
	tx[3]=0x0002;


	DWIN_writeRegiser(tx, 0x00b0, sizeof(tx));
	HAL_Delay(50);

	uint8_t readData[34] = {0};
	DWIN_readRegister(readData, 0x00B4, sizeof(readData));

	uint16_t data[21];

	data[0]=0x5AA5;
	data[1]=0x0000;
	data[2]=0005;
	data[3]=0x0003;

	for(int i=0;i<sizeof(readData)/2;i++)
		data[i+4] = combineBytes(readData[i*2], readData[(i*2)+1]);


    data[9]=picNext;

    DWIN_writeRegiser(data, 0x00b0, sizeof(data));

    HAL_Delay(50);
}

void DWIN_changeKeyboard(uint16_t dil)
{
	if(dil == DW_DIL_RUSCA_VAL)
	{
		keyboard_change_structure(0x001D,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x001E,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x001F,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0020,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0021,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0022,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0023,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0024,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0025,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0026,0x03,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
		keyboard_change_structure(0x0062,0x00,0x0B,0x32,0x46,0x63);
		HAL_Delay(100);
	}
	else if(dil != DW_DIL_ALMANCA_VAL)
	{
		keyboard_change_structure(0x001D,0x03,0x0f,0x23,0x46,0x27); //prescription edit
		HAL_Delay(100);
        keyboard_change_structure(0x001E,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x001F,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0020,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0021,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0022,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0023,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0024,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0025,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);
        keyboard_change_structure(0x0026,0x03,0x0f,0x23,0x46,0x27);
        HAL_Delay(100);

        keyboard_change_structure(0x0062,0x00,0x0f,0x23,0x46,0x27); //bt name edit
        HAL_Delay(100);
	}
	else // almanca
	{
		keyboard_change_structure(0x001D,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x001E,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x001F,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0020,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0021,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0022,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0023,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0024,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0025,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0026,0x03,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
		keyboard_change_structure(0x0062,0x00,0x04,0x32,0x46,0x69);
		HAL_Delay(100);
	}


}

void DWIN_changeWord(uint16_t dil)
{
	switch(dil)
	{
		case DW_DIL_TURKCE_VAL:

			for(int i=0; i<100; i++) //dish names
			{
				DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4000+((i*0x0D)+0x09),4);
			}

			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4514 + 0x09,4); //prescription edit name

			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4521 + 0x09,4); //cooking situation names
			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x452E + 0x09,4);
			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4548 + 0x09,4);

			//DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x453B+0x09,4);  //bt name

			for(int i=0; i<7; i++) //automatic opening names
			{
				DWIN_writeRegiser((uint16_t[]){0x0F0F,0x1428},0x1598+((i*0x3B)+0x09),4);
			}

			//DWIN_writeRegiser(noName_u16,0x17c0,sizeof(noName_u16)); //bt isim


		break;

		case DW_DIL_INGILIZCE_VAL:

			for(int i=0; i<100; i++) //dish names
			{
				DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4000+((i*0x0D)+0x09),4);
			}

			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4514 + 0x09,4); //prescription edit name

			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4521 + 0x09,4); //cooking situation names
			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x452E + 0x09,4);
			DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x4548 + 0x09,4);

			//DWIN_writeRegiser((uint16_t[]){0x0F0F,0x2346},0x453B+0x09,4); //bt name

			for(int i=0; i<7; i++) //automatic opening names
			{
				DWIN_writeRegiser((uint16_t[]){0x0F0F,0x1428},0x1598+((i*0x3B)+0x09),4);
			}

			//DWIN_writeRegiser(noName_u16,0x17c0,sizeof(noName_u16)); 	//bt isim


		break;

		case DW_DIL_RUSCA_VAL:

			for(int i=0; i<100; i++) //dish names
			{
				DWIN_writeRegiser((uint16_t[]){0x0B0B,0x3246},0x4000+((i*0x0D)+0x09),4);
			}

			DWIN_writeRegiser((uint16_t[]){0x0B0B,0x3246},0x4514+0x09,4); //prescription edit name

			DWIN_writeRegiser((uint16_t[]){0x0B0B,0x3246},0x4521 + 0x09,4); //cooking situation names
			DWIN_writeRegiser((uint16_t[]){0x0B0B,0x3246},0x452E + 0x09,4);
			DWIN_writeRegiser((uint16_t[]){0x0B0B,0x3246},0x4548 + 0x09,4);

			//DWIN_writeRegiser((uint16_t[]){0x0B0B,0x3246},0x453B+0x09,4); //bt name

			for(int i=0; i<7; i++) //automatic opening names
			{
				DWIN_writeRegiser((uint16_t[]){0x0606,0x1E28},0x1598+((i*0x3B)+0x09),4);
			}

			//DWIN_writeRegiser(0x17c0,Без_Имени_u16,10); //bt isim

		break;

		case DW_DIL_ALMANCA_VAL:


			for(int i=0; i<100; i++) //dish names
			{
				DWIN_writeRegiser((uint16_t[]){0x0404,0x3246},0x4000+((i*0x0D)+0x09),4);
			}

			DWIN_writeRegiser((uint16_t[]){0x4B04,0x3246},0x4514+0x09,4); //prescription edit name

			DWIN_writeRegiser((uint16_t[]){0x0404,0x3246},0x4521 + 0x09,4); //cooking situation names
			DWIN_writeRegiser((uint16_t[]){0x0404,0x3246},0x452E + 0x09,4);
			DWIN_writeRegiser((uint16_t[]){0x0404,0x3246},0x4548 + 0x09,4);

			//DWIN_writeRegiser((uint16_t[]){0x0404,0x3246},0x453B+0x09,4); //bt name

			for(int i=0; i<7; i++) //automatic opening names
			{
				DWIN_writeRegiser((uint16_t[]){0x0303,0x1E28},0x1598+((i*0x3B)+0x09),4);
			}

			//DWIN_writeRegiser(0x17c0,Без_Имени_u16,10); //bt isim


		break;
	}
}

void dwin_anim_change_structure(uint16_t sp, uint16_t animStop, uint16_t animStart, uint16_t animEnd, uint16_t file)
{
	uint16_t data[4] = {animStop, animStart, animEnd, file};
	DWIN_writeRegiser(data, sp + 0x06, sizeof(data));
}

void DWIN_changeAnim(uint16_t dil)
{
	switch(dil)
	{
		case DW_DIL_TURKCE_VAL:
			dwin_anim_change_structure(0x48B8,0x00C8,0x0002,0x0003,0x3701);
			dwin_anim_change_structure( 0x48C5,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x48D2,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x48DF,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x48EC,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x48F9,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x4906,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x4913,0x00C8, 0x0002, 0x0003, 0x3701);
		break;

		case DW_DIL_INGILIZCE_VAL:
			dwin_anim_change_structure(0x48B8,0x00C8,0x008b,0x008c,0x3a01);
			dwin_anim_change_structure(0x48C5, 0x00C8, 0x008B, 0x008C, 0x3A01);
			dwin_anim_change_structure( 0x48D2,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure( 0x48DF,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure( 0x48EC,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure(0x48F9, 0x00C8, 0x008B, 0x008C, 0x3A01);
			dwin_anim_change_structure(0x4906, 0x00C8, 0x008B, 0x008C, 0x3A01);
			dwin_anim_change_structure(0x4913, 0x00C8, 0x008B, 0x008C, 0x3A01);
		break;
		case DW_DIL_RUSCA_VAL:
			dwin_anim_change_structure(0x48B8,0x00C8,0x008D,0x008E,0x3a01);
			dwin_anim_change_structure( 0x48C5,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure( 0x48D2,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure( 0x48DF,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x48DF,0x00C8, 0x0002, 0x0003, 0x3701);
			dwin_anim_change_structure( 0x48F9,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure( 0x4906,0x00C8, 0x008D, 0x008E, 0x3A01);
			dwin_anim_change_structure( 0x4913,0x00C8, 0x008D, 0x008E, 0x3A01);
		break;
		case DW_DIL_ALMANCA_VAL:
			dwin_anim_change_structure(0x48B8,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x48C5,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x48D2,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x48DF,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x48EC,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x48F9,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x4906,0x00C8,0x0044,0x0045,0x3E01);
			dwin_anim_change_structure(0x4913,0x00C8,0x0044,0x0045,0x3E01);
		break;
	}
}

void DWIN_tcVisualController(uint8_t mode)
{
	uint16_t dataTx[1];
	uint8_t dataRx[2];
	uint16_t temporaryValue;

	if(mode==1) //dual tc
	{
		for(int i=0; i<7; i++)
		{
			DWIN_readRegister(dataRx,0x1595+i*(0x3B),2);
			temporaryValue = (dataRx[0]<<8) | dataRx[1];
			if(temporaryValue==0x0035)
			{
				dataTx[0]=0x1587+(i*0x3B);
				DWIN_writeRegiser(dataTx, 0x15B2+(i*0x3B), sizeof(dataTx));
			}

		}
	}
	else if (mode == 0)  //single tc
	{
		for(int i=0; i<7; i++)
		{
			DWIN_readRegister(dataRx,0x1595+i*(0x3B),2);
			temporaryValue = (dataRx[0]<<8) | dataRx[1];
			if(temporaryValue==0x0035)
			{
				dataTx[0]=0xFF00;
				DWIN_writeRegiser(dataTx, 0x15B2+(i*0x3B), sizeof(dataTx));
			}
		}

	}
	else
	{
		return;
	}
}



void DWIN_dilChange(void)
{
	uint16_t writeData = registerTable[DW_PARAM_DIL_ADR];
	DWIN_writeRegiser(&writeData, DW_DIL_SABIT_YAZI_ADR, sizeof(writeData));

	if(registerTable[DW_PARAM_DIL_ADR] == DW_DIL_ALMANCA_VAL)
	{
		writeData = 1;
		DWIN_writeRegiser(&writeData, 0x843B, sizeof(writeData));
	}
	else
	{
		writeData = 0;
		DWIN_writeRegiser(&writeData, 0x843B, sizeof(writeData));
	}

	DWIN_changePopup(registerTable[DW_PARAM_DIL_ADR]);

	DWIN_changeIcon(registerTable[DW_PARAM_DIL_ADR]);

	DWIN_changeWord(registerTable[DW_PARAM_DIL_ADR]);

	DWIN_changeKeyboard(registerTable[DW_PARAM_DIL_ADR]);

	DWIN_changeAnim(registerTable[DW_PARAM_DIL_ADR]);

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

DWIN_Response DWIN_buzzerSet(uint8_t setLevel)
{
	DWIN_Response response = NO_RESPONSE;

	if(setLevel)
	{
		uint16_t writeData[2] = {0x5AFF,0x90B9};
		response = DWIN_writeRegiser(writeData, 0x0080, sizeof(writeData));
	}
	else
	{
		uint16_t writeData[2] = {0x5AFF,0x90B1};
		response = DWIN_writeRegiser(writeData, 0x0080, sizeof(writeData));
	}

	return response;
}




DWIN_Response touchSetOnOff_structure(uint16_t mode, uint16_t page , uint8_t controlID, uint8_t keyCode)
{
	DWIN_Response response = NO_RESPONSE;

	uint16_t tx[4];

	tx[0]=0x5AA5;
	tx[1]=page;
	tx[2]=(uint16_t)(controlID<<0x08)+keyCode;
	tx[3]=mode;

	HAL_Delay(50);

	response = DWIN_writeRegiser(tx, 0x00b0, sizeof(tx));

	return response;
}

DWIN_Response DWIN_dokunmatik_msg(void)
{
	DWIN_Response response = NO_RESPONSE;

	touchSetOnOff_structure(1,0,0,5);
	touchSetOnOff_structure(1,0,1,2);
	touchSetOnOff_structure(1,0,2,5);
	touchSetOnOff_structure(1,0,3,5);

    // Pişirme sonrası alarmın dokunmatikleri
	touchSetOnOff_structure(0,82,0,5);			// Recete - cift tc buhar var
	touchSetOnOff_structure(0,83,0,5);			// Recete - cift tc buhar yok
	touchSetOnOff_structure(0,84,0,5);			// Recete - tek tc buhar var
	touchSetOnOff_structure(0,85,0,5);			// Recete - tek tc buhar yok

	touchSetOnOff_structure(0,2,0,5);			// Manuel - cift tc buhar var
	touchSetOnOff_structure(0,96,0,5);			// Manuel - cift tc buhar yok
	touchSetOnOff_structure(0,94,0,5);			// Manuel - tek tc buhar var
	touchSetOnOff_structure(0,95,0,5);			// Manuel - tek tc buhar yok


	return response;
}


void DWIN_run(void)
{
	if((HAL_GetTick() - counterTick.run) >= 1000) // manuel pisirme registerını periyodik olarak oku mesaj kaçırabilir
	{
		counterTick.run = HAL_GetTick();

		HAL_GPIO_TogglePin(RUN_LED);

		UserApp_IWDG_Refresh();

		calculate_temperature();

		if(DWIN.Init == 1)
		{

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

			else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_CIHAZ_TEST_SAYFA_ENTER)
			{
				uint8_t in1,in2,in3,in4,in5,in6,in_AC_1 = 0,in_AC_2 = 0;

				in1 	= HAL_GPIO_ReadPin(INPUT_1);
				in2 	= HAL_GPIO_ReadPin(INPUT_2);
				in3 	= HAL_GPIO_ReadPin(INPUT_3);
				in4 	= HAL_GPIO_ReadPin(INPUT_4);
				in5 	= HAL_GPIO_ReadPin(INPUT_5);
				in6 	= HAL_GPIO_ReadPin(INPUT_6);

				for(int i=0;i<25;i++)
				{
					if(HAL_GPIO_ReadPin(INPUT_AC_1) == 1)
						in_AC_1 = 1;

					if(HAL_GPIO_ReadPin(INPUT_AC_2) == 1)
						in_AC_2 = 1;

					HAL_Delay(0);
				}

				uint16_t testSendData[11] = {temp.TC1,temp.TC2,temp.TC3,
											in1,in2,in3,in4,in5,in6,in_AC_1,in_AC_2};

				DWIN_writeRegiser(testSendData, DW_TEST_TC1_ADR,sizeof(testSendData));
			}

		}

	}

	shiftRefresh();

	DWIN_check();
	DWIN_answerProcess();
	DWIN_manuelPisirmeSuresi();
	DWIN_manuelBuharSuresi();
	DWIN_pisirmeSonuAlarm();
	DWIN_lambaSuresi();
	DWIN_buharHazirCheck();
	DWIN_otomatikPisirmeBaslatmaCheck();
}


static uint8_t dwin_check_counter = 0;

void DWIN_check(void)
{
	if((HAL_GetTick() - counterTick.dwinCheck) >= 500)
	{
		if((DWIN.Init != 1) && (dwin_check_counter < 5))
		{
			uint8_t version[2];

			if(DWIN_readRegister(version, DWIN_VERSION_ADDR, sizeof(version)) == READ_OK)
			{
				dwin_check_counter = 0;

				SEGGER_RTT_printf(0,"DWIN OK ! Version : %x%x\r\n",version[0],version[1]);

				///////////////////////////////////////////////////////////////////
				uint8_t saniye,dakika,saat,gun,hafta,ay,yil;
				DWIN_readRTC(&saniye, &dakika, &saat, &hafta, &gun, &ay, &yil);
				RTC_SetDateTime(saat, dakika, saniye, gun, ay, yil);
				///////////////////////////////////////////////////////////////////

				if(registerTable[DW_PARAM_DIL_ADR] != DW_DIL_TURKCE_VAL)
					DWIN_dilChange();


				for(int i=0;i<EEPROM_TABLE_LEN;i++)
				{
					uint16_t writeDwin = registerTable[eepromAddrTable[i]];
					DWIN_writeRegiser(&writeDwin, eepromAddrTable[i], sizeof(writeDwin));
				}

				if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
				{

					DWIN_change_buhar_settings(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR]);

				}

				if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
				{
					DWIN_change_cihaz_type_settings(registerTable[DW_PARAM_CIHAZ_TYPE_ADR]);
				}

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

				DWIN_resetManuelPisirme();

				uint16_t writeData = 0;

				if(registerTable[DW_PARAM_BUTTON_SOUND_ADR] == 1)
				{
					DWIN_buzzerSet(1);
					writeData = registerTable[DW_PARAM_BUTTON_SOUND_ADR];
					DWIN_writeRegiser(&writeData, DW_PARAM_BUTTON_SOUND_ADR, sizeof(writeData));
				}
				else
				{
					DWIN_buzzerSet(0);
					writeData = registerTable[DW_PARAM_BUTTON_SOUND_ADR];
					DWIN_writeRegiser(&writeData, DW_PARAM_BUTTON_SOUND_ADR, sizeof(writeData));
				}


				DWIN_dokunmatik_msg();

				writeData = 0;

				DWIN_writeRegiser(&writeData, DW_LOADING_PAGE_ADR, sizeof(writeData));

				DWIN_changePage(0);

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

			HAL_GPIO_WritePin(ESP32_EN, 0);
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
				registerTable[DW_SURE_SONU_ALARM_ANIM_ADR] = 1;

				if(registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)
				{
					if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
					{
						if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
							touchSetOnOff_structure(1,95,0,5);			// Manuel - tek tc buhar yok

						else
							touchSetOnOff_structure(1,94,0,5);			// Manuel - tek tc buhar var
					}
					else
					{
						if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
							touchSetOnOff_structure(1,96,0,5);			// Manuel - cift tc buhar yok

						else
							touchSetOnOff_structure(1,2,0,5);			// Manuel - cift tc buhar var
					}
				}
				else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
				{
					if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
					{
						if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
							touchSetOnOff_structure(1,85,0,5);			// Recete - tek tc buhar yok

						else
							touchSetOnOff_structure(1,84,0,5);			// Recete - tek tc buhar var
					}
					else
					{
						if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
							touchSetOnOff_structure(1,83,0,5);			// Recete - cift tc buhar yok

						else
							touchSetOnOff_structure(1,82,0,5);			// Recete - cift tc buhar var
					}
				}

				uint16_t data = 1;
				DWIN_writeRegiser(&data, DW_SURE_SONU_ALARM_ANIM_ADR, sizeof(data));

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
		if((HAL_GetTick() - counterTick.lambaSuresi) >= (registerTable[DW_PARAM_LAMBA_SURESI_ADR] * 1000))
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

			else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_OTOMATIK_ACMA_SAYFA_ENTER)
				DWIN_otomatikSayfa();

			else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_CIHAZ_TEST_SAYFA_ENTER)
			{
				DWIN_testSayfa();
			}

		}

	}
}

void DWIN_testSayfa(void)
{
	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);
	uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

	if((addr >= DW_TEST_K1_ADR)&&(addr <= DW_TEST_BUZZER_ADR))
	{
		uint32_t outputList[18] = {K1,K2,K3,K4,K5,K6,K8,K9,K10,K11,K12,K13,K14,K15,K16,K17,K18,BUZZER};

		setOut(outputList[addr-DW_TEST_K1_ADR],data);
	}
	else
	{
		switch(addr)
		{
			case DW_TEST_HEPSINIAC_ADR:

				if(data == 1)
				{
					setOut(K1|K2|K3|K4|K5|K6|K8|K9|K10|K11|K12|K13|K14|K15|K16|K17|K18,1);

					uint16_t writeData[17] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
					DWIN_writeRegiser(writeData, DW_TEST_K1_ADR, sizeof(writeData));
				}

			break;

			case DW_TEST_HEPSINIKAPAT_ADR:

				setOut(K1|K2|K3|K4|K5|K6|K8|K9|K10|K11|K12|K13|K14|K15|K16|K17|K18|BUZZER,0);

				uint16_t writeData2[19] = {0};
				DWIN_writeRegiser(writeData2, DW_TEST_K1_ADR, sizeof(writeData2));

			break;

			case DW_TEST_EXIT_ADR:

				registerTable[REG_DW_MODE_INFO_ADR] = 0;

				setOut(K1|K2|K3|K4|K5|K6|K8|K9|K10|K11|K12|K13|K14|K15|K16|K17|K18|BUZZER,0);
				PWM_StartSmoothTransition(0, 50);
				setAnalogVoltage(0, DAC_CHANNEL_1);
				setAnalogVoltage(0, DAC_CHANNEL_2);

				uint16_t writeData3[30] = {0};
				DWIN_writeRegiser(writeData3, DW_TEST_K1_ADR, sizeof(writeData3));

			break;
		}
	}

	if((addr >= DW_TEST_1KHZ_ADR)&&(addr <= DW_TEST_4KHZ_ADR))
	{
		uint16_t writeData[4] = {0};

		writeData[addr - DW_TEST_1KHZ_ADR] = data;

		PWM_StartSmoothTransition((addr - DW_TEST_1KHZ_ADR + 1)*1000*data, 50);

		DWIN_writeRegiser(writeData, DW_TEST_1KHZ_ADR, sizeof(writeData));
	}
	else if((addr >= DW_TEST_OUT2_3V_ADR)&&(addr <= DW_TEST_OUT2_9V_ADR))
	{
		uint16_t writeData[3] = {0};

		setAnalogVoltage((addr - DW_TEST_OUT2_3V_ADR + 1) * 3.0 * data , DAC_CHANNEL_2);

		writeData[addr - DW_TEST_OUT2_3V_ADR] = data;

		DWIN_writeRegiser(writeData, DW_TEST_OUT2_3V_ADR, sizeof(writeData));
	}
	else if((addr >= DW_TEST_OUT1_5V_ADR)&&(addr <= DW_TEST_OUT1_15V_ADR))
	{
		uint16_t writeData[3] = {0};

		setAnalogVoltage((addr - DW_TEST_OUT1_5V_ADR + 1) * 5.0 * data , DAC_CHANNEL_1);

		writeData[addr - DW_TEST_OUT1_5V_ADR] = data;

		DWIN_writeRegiser(writeData, DW_TEST_OUT1_5V_ADR, sizeof(writeData));
	}
}


void DWIN_change_buhar_settings(uint16_t setVal)
{

	for (int i = 0; i < 10; i++)
	{
		touchSetOnOff_structure(((uint8_t) (setVal & 0x0F)), 0x001d + i,0x01, 0x08);
		HAL_Delay(50);
	}

	for(int i=0; i<8; i++)
	{
		touchSetOnOff_structure(((uint8_t) (setVal & 0x0F)), 0x0035 + i,0x01, 0x02);
		HAL_Delay(50);
	}

	uint16_t writeData = !setVal;
	DWIN_writeRegiser(&writeData, 0x7AAB, sizeof(writeData));

}

void DWIN_change_cihaz_type_settings(uint16_t setVal)
{
	uint16_t writeData = !setVal;
	DWIN_writeRegiser(&writeData, 0x7AAA, sizeof(writeData));

	for (int i = 0; i < 10; i++)
	{
		touchSetOnOff_structure(((uint8_t)(setVal>>4)),0x001d+i,0x00,0x08);
	}

	DWIN_writeRegiser(&writeData, 0x7AAC, sizeof(writeData));

	touchSetOnOff_structure(setVal,0x0035,00,05);
	touchSetOnOff_structure(setVal,0x0037,00,05);
	touchSetOnOff_structure(setVal,0x0038,00,05);
	touchSetOnOff_structure(setVal,0x0039,00,05);

	DWIN_tcVisualController(setVal);

	if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0) //if single temperature control is active, change the icons to match the single temperature control mode
	{
		switch(registerTable[DW_PARAM_DIL_ADR])
		{
			case 0x000:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure(0x4868+(i*0x0a), 0x009F,0x009F, 0x3A01);
				}

				break;
			case 0x001:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure(0x4868+(i*0x0a), 0x009D,0x009D, 0x3A01);
				}

				break;
			case 0x002:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure(0x4868+(i*0x0a), 0x009e,0x009e, 0x3A01);
				}

				break;
			case 0x003:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure( 0x4868+(i*0x0a), 0x0043, 0x0043, 0x3E01);
				}

				break;
		}
	}
	else
	{
		switch(registerTable[DW_PARAM_DIL_ADR])

		{
			case 0x000:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure( 0x4868+(i*0x0a), 0x0016, 0x0016, 0x3700);
				}

				break;
			case 0x001:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure(0x4868+(i*0x0a), 0x0089, 0x0089, 0x3A00);
				}

				break;
			case 0x002:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure( 0x4868+(i*0x0a), 0x008a, 0x008a, 0x3A00);
				}

				break;
			case 0x003:

				for(int i=0; i<7; i++)
				{
					dwin_icon_change_structure( 0x4868+(i*0x0a), 0x004B, 0x004B, 0x3E01);
				}

				break;
		}
	}
}

void DWIN_anaSayfa(void)
{
	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);
	uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);
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


		break;

		case DW_MANUEL_MOD_GIRIS_ADR:

			if(data == 0)
			{
				DWIN_resetManuelPisirme();
			}


			else if(data == 1)
			{
				registerTable[REG_DW_MODE_INFO_ADR] = DW_MANUEL_MODE_ENTER;

				setOut(K14, data);
				PID_Setup();

			}


		break;

		case DW_RECETE_SAYFA_ENTER_ADR:


			if(data == 0)
				registerTable[REG_DW_MODE_INFO_ADR] = DW_ANA_SAYFA_ENTER;

			if(data == 1)
				registerTable[REG_DW_MODE_INFO_ADR] = DW_RECETE_SAYFA_ENTER;


		break;

		case DW_PARAMETRE_PAGE_ADR:

			if(data == registerTable[DW_PARAM_PSW_ADR])
			{
				DWIN_changePage(DW_AYARLAR_PAGE_ADR);
			}

			else if(data == DW_CIHAZ_TEST_PAGE_PSW)
			{
				outputData = 0;
				DWIN_changePage(DW_CIHAZ_TEST_PAGE_ADR);
				registerTable[REG_DW_MODE_INFO_ADR] = DW_CIHAZ_TEST_SAYFA_ENTER;
			}

			else
				DWIN_changePage(DW_ANA_PAGE_ADR);


		break;

		case DW_PARAMETRE_EXIT_PAGE_ADR:

			uint8_t pageChangeCheck = 0;

			uint8_t readData[2];
			DWIN_readRegister(readData, DW_PARAM_DIL_ADR, sizeof(readData));

			if(readData[1] != registerTable[DW_PARAM_DIL_ADR])
			{
				registerTable[DW_PARAM_DIL_ADR] = readData[1];

				EEPROM_Write(&hi2c1, DW_PARAM_DIL_ADR, readData, sizeof(readData));

				DWIN_changePage(DW_EMPTY_PAGE_NUM);

				uint16_t writeData = 1;
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

				registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] = readData[1];

				EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_ACTIVE_ADR, readData, sizeof(readData));

				DWIN_changePage(DW_EMPTY_PAGE_NUM);

				uint16_t writeData = 1;
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

				registerTable[DW_PARAM_CIHAZ_TYPE_ADR] = readData[1];

				EEPROM_Write(&hi2c1, DW_PARAM_CIHAZ_TYPE_ADR, readData, sizeof(readData));

				DWIN_changePage(DW_EMPTY_PAGE_NUM);

				uint16_t writeData = 1;
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

		case DW_PARAM_LAMBA_SURESI_ADR:

			registerTable[DW_PARAM_LAMBA_SURESI_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_LAMBA_SURESI_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_BUTTON_SOUND_ADR:

			registerTable[DW_PARAM_BUTTON_SOUND_ADR] = data;

			if(data == 1)
				DWIN_buzzerSet(1);
			else
				DWIN_buzzerSet(0);

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUTTON_SOUND_ADR, data2, sizeof(data2));


		break;

		case DW_PARAM_ALARM_ADR:

			registerTable[DW_PARAM_ALARM_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_ALARM_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_PSW_ADR:

			registerTable[DW_PARAM_PSW_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_PSW_ADR, data2, sizeof(data2));

			DWIN_changePage(DW_AYARLAR_PAGE_ADR);

		break;


//		case DW_PARAM_BUHAR_ACTIVE_ADR:
//
//			registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] = data;
//
//			//DWIN_buharActivePassive(data);
//
//			parse16BitTo8Bit(data, &data2[0], &data2[1]);
//
//			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_ACTIVE_ADR, data2, sizeof(data2));
//
//
//		break;

		case DW_PARAM_BUHAR_SENSOR_TYPE_ADR:

			registerTable[DW_PARAM_BUHAR_SENSOR_TYPE_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_SENSOR_TYPE_ADR, data2, sizeof(data2));

		break;


		case DW_PARAM_BUHAR_MAX_SET_ADR:

			registerTable[DW_PARAM_BUHAR_MAX_SET_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_MAX_SET_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_BUHAR_HAZIR_SICAK_ADR:

			registerTable[DW_PARAM_BUHAR_HAZIR_SICAK_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_HAZIR_SICAK_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_BUHAR_UST_HIS_ADR:

			registerTable[DW_PARAM_BUHAR_UST_HIS_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_UST_HIS_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_BUHAR_ALT_HIS_ADR:

			registerTable[DW_PARAM_BUHAR_ALT_HIS_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_BUHAR_ALT_HIS_ADR, data2, sizeof(data2));

		break;

		case DW_PARAM_TERMOKUPL_TYPE_ADR:

			registerTable[DW_PARAM_TERMOKUPL_TYPE_ADR] = data;

			parse16BitTo8Bit(data, &data2[0], &data2[1]);

			EEPROM_Write(&hi2c1, DW_PARAM_TERMOKUPL_TYPE_ADR, data2, sizeof(data2));

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

		case DW_TARIH_SAAT_PAGE_ENTER_ADR:


			if(data == 1)
			{
				uint8_t saniye, dakika, saat, gun, hafta ,ay, yil;

				DWIN_readRTC(&saniye, &dakika, &saat, &hafta, &gun, &ay, &yil);
				DWIN_writeRTC(saniye, dakika, saat, gun, ay, yil, 0);
			}

		break;

		case DW_OTOMATIK_ACMA_ENTER_ADR:

			if(data == 1)
				registerTable[REG_DW_MODE_INFO_ADR] = DW_OTOMATIK_ACMA_SAYFA_ENTER;
			else
				registerTable[REG_DW_MODE_INFO_ADR] = DW_ANA_SAYFA_ENTER;

		break;

		case DW_FIRST_WRITE_RTC_ADR:

			if(data == DW_WRITE_RTC_DONE_MSG)
			{
				HAL_Delay(1000);
				uint8_t saniye,dakika,saat,gun,hafta,ay,yil;
				DWIN_readRTC(&saniye, &dakika, &saat, &hafta, &gun, &ay, &yil);
				RTC_SetDateTime(saat, dakika, saniye, gun, ay, yil);
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


		case DW_ARIZA_ALARM_SUSTURMA_ADR:

			data = 1;

			registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR] = 1;

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);
			setOut(K10, 0);

		break;

	}
}
void DWIN_manuelSayfa(void)
{

	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);
	uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

	switch(addr)
	{



		case DW_BUHAR_HAZIRLAMA_ADR:

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

		break;

		case DW_UST_SICAKLIK_SET_ONAY_ADR:


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

		case DW_PISIRME_SURESI_SET_ONAY_ADR:


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
				registerTable[DW_SURE_SONU_ALARM_ANIM_ADR] 	= 0;

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


				if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						DWIN_changePage(DW_PAGE_RECETE_BUHARYOK_TEKTC_ADR);			// Manuel - tek tc buhar yok

					else
						DWIN_changePage(DW_PAGE_RECETE_BUHARVAR_TEKTC_ADR);			// Manuel - tek tc buhar var
				}
				else
				{
					if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
						DWIN_changePage(DW_PAGE_RECETE_BUHARYOK_CIFTTC_ADR);			// Manuel - cift tc buhar yok

					else
						DWIN_changePage(DW_PAGE_RECETE_BUHARVAR_CIFTTC_ADR);			// Manuel - cift tc buhar var
				}

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

void automaticOpeningVisualController(uint8_t dayNumber, uint8_t mode, uint8_t tcCount) //mode 1: manual, mode 2: recipe, 1-7: day number
{
    uint16_t baseTextSp=0x1598;
    uint16_t baseIconVp=0x1595;
    uint16_t baseTopTempSp=0x15A5;
    uint16_t baseBottomTempSp=0x15B2;
    uint16_t baseTopTempVp=0x1586;
    uint16_t baseBottomTempVp=0x1587;
    uint16_t baseTextVp=0x1589;

    uint16_t addrOffset = (dayNumber - 1) * (0x3B);

    uint16_t data[1];

    if(tcCount==0x01) //dual tc
    {
        if(mode==1) //manual
        {
            // delete text
            data[0] = 0xFF00;
            DWIN_writeRegiser(data, baseTextSp + addrOffset, sizeof(data));

            // activate icon
            data[0] = 0x0035;
            DWIN_writeRegiser(data, baseIconVp + addrOffset, sizeof(data));

            // activate top temp value
            data[0] = baseTopTempVp + addrOffset;
            DWIN_writeRegiser(data, baseTopTempSp + addrOffset, sizeof(data));

            // activate bottom temp value
            data[0] = baseBottomTempVp + addrOffset;
            DWIN_writeRegiser(data, baseBottomTempSp + addrOffset, sizeof(data));
        }
        else if (mode == 2) //recipe
        {
            // activate text
            data[0] = baseTextVp + addrOffset;
            DWIN_writeRegiser(data, baseTextSp + addrOffset, sizeof(data));

            // deactivate icon
            data[0] = 0x003A;
            DWIN_writeRegiser(data, baseIconVp + addrOffset, sizeof(data));

            // delete top temp value
            data[0] = 0xFF00;
            DWIN_writeRegiser(data, baseTopTempSp + addrOffset, sizeof(data));

            // delete bottom temp value
            data[0] = 0xFF00;
            DWIN_writeRegiser(data, baseBottomTempSp + addrOffset, sizeof(data));
        }
        else
        {
            return;
        }

    }
    else if(tcCount==0x00) //single tc
    {
        if(mode==1) //manual
        {
            // delete text
            data[0] = 0xFF00;
            DWIN_writeRegiser(data, baseTextSp + addrOffset, sizeof(data));

            // activate icon
            data[0] = 0x0035;
            DWIN_writeRegiser(data, baseIconVp + addrOffset, sizeof(data));

            // activate top temp value
            data[0] = baseTopTempVp + addrOffset;
            DWIN_writeRegiser(data, baseTopTempSp + addrOffset, sizeof(data));

        }
        else if (mode == 2) //recipe
        {
            // activate text
            data[0] = baseTextVp + addrOffset;
            DWIN_writeRegiser(data, baseTextSp + addrOffset, sizeof(data));

            // deactivate icon
            data[0] = 0x003A;
            DWIN_writeRegiser(data, baseIconVp + addrOffset, sizeof(data));

            // delete top temp value
            data[0] = 0xFF00;
            DWIN_writeRegiser(data, baseTopTempSp + addrOffset, sizeof(data));

        }
        else
        {
            return;
        }
    }
    else {
		return;
	}


    //written by Senior Embedded Software Engineer Hakan Altunkanat on 13.04.2026
    //All rights reserved by Step Elektronik San. ve Tic. Ltd. Sti.
}

void DWIN_otomatikSayfa(void)
{
	uint16_t addr = combineBytes(DWIN_rxBuffer[4], DWIN_rxBuffer[5]);
	uint16_t data = combineBytes(DWIN_rxBuffer[7], DWIN_rxBuffer[8]);

	if((addr >= DW_OTOMATIK_AKTIF_INFO_ADR)&&(addr <= (DW_OTOMATIK_AKTIF_INFO_ADR + (DW_OTOMATIK_ACMA_ADR_LENGTH*6))))
		islemdekiOtomatikAktifIkon = (addr - DW_OTOMATIK_AKTIF_INFO_ADR) / DW_OTOMATIK_ACMA_ADR_LENGTH;

	else
	{
		switch(addr)
		{
			case DW_OTOTMATIK_GUN_ENTER_ADR:

				uint16_t gun_data = combineBytes(DWIN_rxBuffer[37], DWIN_rxBuffer[38]);

				if((gun_data > 0)&&((gun_data <= 7)))
					islemdekiOtomatikGun = gun_data;

			break;

			case DW_OTOTMATIK_GUN_EXIT_ADR:

				// otomatik parametreler okunup eeprom ve registerTable yazılacak

				if((data == 1)||(data == 2))
				{
//
					uint8_t readData[EE_OTOMATIK_ACMA_PARAM_SIZE - 4 + DW_RECETE_ISIM_SIZE];
					DWIN_readRegister(readData, DW_OTOTMATIK_GUN_ENTER_ADR, sizeof(readData));

					uint16_t writeData[sizeof(readData)/2];

					convert_u8_to_u16(readData, writeData, sizeof(readData));

					DWIN_writeRegiser(writeData, DW_OTOMATIK_ACMA_ILK_ADR + ((islemdekiOtomatikGun-1) * DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(writeData));

					automaticOpeningVisualController(islemdekiOtomatikGun, data, registerTable[DW_PARAM_CIHAZ_TYPE_ADR]);



					for(int i=0;i<sizeof(writeData)/2;i++)
						registerTable[DW_OTOMATIK_ACMA_ILK_ADR + ((islemdekiOtomatikGun-1) * DW_OTOMATIK_ACMA_ADR_LENGTH) + i] = writeData[i];


					registerTable[DW_OTOMATIK_PISIRME_INFO_ADR + ((islemdekiOtomatikGun-1)*DW_OTOMATIK_ACMA_ADR_LENGTH)] = data; // manuel/recete bilgisi

					uint16_t Otomatik_param[EE_OTOMATIK_ACMA_PARAM_SIZE/2] = {0};

					for(int i=0;i<(sizeof(Otomatik_param)/2) - 2;i++)
						Otomatik_param[i] = writeData[i];

					Otomatik_param[(EE_OTOMATIK_ACMA_PARAM_SIZE/2) - 3] = registerTable[DW_OTOMATIK_PISIRME_INFO_ADR + ((islemdekiOtomatikGun-1)*DW_OTOMATIK_ACMA_ADR_LENGTH)]; // manuel/recete bilgisi
					Otomatik_param[(EE_OTOMATIK_ACMA_PARAM_SIZE/2) - 2] = registerTable[DW_OTOMATIK_AKTIF_INFO_ADR + ((islemdekiOtomatikGun-1)*DW_OTOMATIK_ACMA_ADR_LENGTH)];

					uint8_t isim_row_data[2];
					uint16_t isim_row_data_u16;

					DWIN_readRegister(isim_row_data, DW_OTOMATIK_ACMA_ISIM_CHOOSE, sizeof(isim_row_data));

					isim_row_data_u16 = combineBytes(isim_row_data[0], isim_row_data[1]);

					Otomatik_param[(EE_OTOMATIK_ACMA_PARAM_SIZE/2) - 1] = isim_row_data_u16 - 1;

					uint8_t Otomatik_param_u8[sizeof(Otomatik_param)];

					convert_u16_to_u8(Otomatik_param, Otomatik_param_u8, sizeof(Otomatik_param));

					EEPROM_Write(&hi2c1, EE_OTOMATIK_ACMA_ILK_ADR + ((islemdekiOtomatikGun-1)*(EE_OTOMATIK_ACMA_PARAM_SIZE)), Otomatik_param_u8, sizeof(Otomatik_param_u8));

					islemdekiOtomatikGun = 0;
				}



			break;

			case DW_OTOMATIK_ACMA_ISIM_CHOOSE:

				uint8_t recete_isim_data_u8[DW_RECETE_ISIM_SIZE];
				EEPROM_Read_Safe(&hi2c1, EE_RECETE_ILK_ADR + EE_RECETE_ISIM_ROW +((data-1)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_isim_data_u8, sizeof(recete_isim_data_u8));

				uint16_t recete_isim_data_u16[DW_RECETE_ISIM_SIZE/2];

				convert_u8_to_u16(recete_isim_data_u8, recete_isim_data_u16, sizeof(recete_isim_data_u8));

				DWIN_writeRegiser(recete_isim_data_u16, DW_OTOMATIK_ISIM_ROW_ADR, sizeof(recete_isim_data_u16));

			break;

			case DW_OTOTMATIK_IKON_AKTIF_ADR:

				if(data == 1)
				{

					uint8_t saatIkonCheck = 0;

					for(int i=0;i<7;i++)
					{
						if(registerTable[0x1597 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] == 1)
						{
							saatIkonCheck = 1;
							break;
						}
					}

					if(saatIkonCheck == 0)
					{
						uint16_t ikonWrite = 1;
						DWIN_writeRegiser(&ikonWrite, DW_SAAT_IKON_ADDR, sizeof(ikonWrite));
						registerTable[DW_SAAT_IKON_ADDR] = 1;
					}

					uint16_t oto_write = 0x003E;
					DWIN_writeRegiser(&oto_write, 0x1597 + (islemdekiOtomatikAktifIkon*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));

					registerTable[0x1597 + (islemdekiOtomatikAktifIkon*DW_OTOMATIK_ACMA_ADR_LENGTH)] = 1;

					uint8_t writeData = 1;

					EEPROM_Write(&hi2c1, EE_OTOMATIK_ACMA_ILK_ADR + 13 + (islemdekiOtomatikAktifIkon*EE_OTOMATIK_ACMA_PARAM_SIZE), &writeData, sizeof(writeData));

				}

				else if(data == 0)
				{
					uint16_t oto_write = 0x003D;
					DWIN_writeRegiser(&oto_write, 0x1597 + (islemdekiOtomatikAktifIkon*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));

					registerTable[0x1597 + (islemdekiOtomatikAktifIkon*DW_OTOMATIK_ACMA_ADR_LENGTH)] = 0;

					uint8_t writeData = 0;

					EEPROM_Write(&hi2c1, EE_OTOMATIK_ACMA_ILK_ADR + 13 + (islemdekiOtomatikAktifIkon*EE_OTOMATIK_ACMA_PARAM_SIZE), &writeData, sizeof(writeData));

					uint8_t saatIkonCheck = 0;

					for(int i=0;i<7;i++)
					{
						if(registerTable[0x1597 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] == 1)
						{
							saatIkonCheck = 1;
							break;
						}
					}

					if(saatIkonCheck == 0)
					{
						uint16_t ikonWrite = 0;
						DWIN_writeRegiser(&ikonWrite, DW_SAAT_IKON_ADDR, sizeof(ikonWrite));
						registerTable[DW_SAAT_IKON_ADDR] = 0;
					}

				}

				islemdekiOtomatikAktifIkon = 0;

			break;

		}
	}

}

void DWIN_otomatikAcmaCheck(void)
{
	if((registerTable[DW_SAAT_IKON_ADDR] == 1)&&(registerTable[REG_DW_MODE_INFO_ADR] == DW_ANA_SAYFA_ENTER))
	{
	    RTC_TimeTypeDef sTime = {0};
	    RTC_DateTypeDef sDate = {0};

	    RTC_GetDateTime(&sTime, &sDate);

		for(int i=0;i<7;i++)
		{
			if(registerTable[DW_OTOMATIK_AKTIF_INFO_ADR + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] == 1)
			{

			    if((sDate.WeekDay - 1) == i)
			    {
			    	if(	(registerTable[DW_OTOMATIK_ACMA_SAAT_ADR + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] == sTime.Hours) &&
			    		(registerTable[DW_OTOMATIK_ACMA_MIN_ADR + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] == sTime.Minutes)&&
						sTime.Seconds == 0)
			    	{

			    		otomatikPisirmeBaslatmaFlag = sDate.WeekDay;
//			    	    SEGGER_RTT_printf(0,"Saat: %02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
//			    	    SEGGER_RTT_printf(0,"Tarih: %02d/%02d/20%02d  gun:%d\n", sDate.Date, sDate.Month, sDate.Year,sDate.WeekDay);
			    	}
			    }

			}
		}
	}
}

void DWIN_otomatikPisirmeBaslatmaCheck(void)
{
	if(otomatikPisirmeBaslatmaFlag > 0)
	{
		uint8_t eeprom_row = otomatikPisirmeBaslatmaFlag - 1;
		uint16_t otomatikPisirmeInfo 	= registerTable[DW_OTOMATIK_PISIRME_INFO_ADR + (eeprom_row*DW_OTOMATIK_ACMA_ADR_LENGTH)];
		uint16_t otomatikPisirmeBuhar 	= registerTable[DW_OTOMATIK_BUHAR_ADR + (eeprom_row*DW_OTOMATIK_ACMA_ADR_LENGTH)];

		switch(otomatikPisirmeInfo)
		{
			case DW_OTOMATIK_MANUEL_PISIRME:

				uint8_t ustSicaklik_u8[2];
				uint16_t ustSicaklik_u16 = registerTable[DW_OTOMATIK_UST_SICAKLIK_ADR + (eeprom_row*DW_OTOMATIK_ACMA_ADR_LENGTH)];

				uint8_t altSicaklik_u8[2];
				uint16_t altSicaklik_u16 = registerTable[DW_OTOMATIK_ALT_SICAKLIK_ADR + (eeprom_row*DW_OTOMATIK_ACMA_ADR_LENGTH)];


				registerTable[REG_DW_MODE_INFO_ADR] = DW_MANUEL_MODE_ENTER;

				registerTable[DW_UST_SICAKLIK_SET_ADR] = ustSicaklik_u16;
				registerTable[DW_ALT_SICAKLIK_SET_ADR] = altSicaklik_u16;

				parse16BitTo8Bit(ustSicaklik_u16, &ustSicaklik_u8[0], &ustSicaklik_u8[1]);
				parse16BitTo8Bit(altSicaklik_u16, &altSicaklik_u8[0], &altSicaklik_u8[1]);

				setOut(K14, 1);
				//DWIN_enterManuelProcess();
				PID_Setup();


				DWIN_writeRegiser(&ustSicaklik_u16, DW_UST_SICAKLIK_SET_ADR, sizeof(ustSicaklik_u16));
				DWIN_writeRegiser(&altSicaklik_u16, DW_ALT_SICAKLIK_SET_ADR, sizeof(altSicaklik_u16));

				DWIN_changePage(DW_PISIRME_PAGE_ADR);

				EEPROM_Write(&hi2c1, DW_UST_SICAKLIK_SET_ADR, ustSicaklik_u8, sizeof(ustSicaklik_u8));
				EEPROM_Write(&hi2c1, DW_ALT_SICAKLIK_SET_ADR, altSicaklik_u8, sizeof(altSicaklik_u8));

				if(otomatikPisirmeBuhar == 1)
				{
					uint16_t buharData = 1;
					setOut(K8, 1);
					registerTable[DW_BUHAR_HAZIR_ANIM] = buharData;
					registerTable[DW_BUHAR_HAZIRLAMA_ADR] = buharData;

					DWIN_writeRegiser(&buharData, DW_BUHAR_HAZIR_ANIM, sizeof(buharData));
					DWIN_writeRegiser(&buharData, DW_BUHAR_HAZIRLAMA_ADR, sizeof(buharData));
				}


			break;

			case DW_OTOMATIK_RECETE_PISIRME:

				if(otomatikPisirmeBuhar == 1)
				{
					uint16_t buharData = 1;
					setOut(K8, 1);
					registerTable[DW_BUHAR_HAZIR_ANIM] = buharData;
					registerTable[DW_BUHAR_HAZIRLAMA_ADR] = buharData;

					DWIN_writeRegiser(&buharData, DW_BUHAR_HAZIR_ANIM, sizeof(buharData));
					DWIN_writeRegiser(&buharData, DW_BUHAR_HAZIRLAMA_ADR, sizeof(buharData));
				}

				uint8_t fakeRxBuffer[11] = {0};

				fakeRxBuffer[0] = 0x5A;
				fakeRxBuffer[1] = 0xA5;
				fakeRxBuffer[2] = 0x08;
				fakeRxBuffer[3] = 0x83;

				uint8_t readData[2];
				EEPROM_Read(&hi2c1, EE_OTOMATIK_RECETE_ROW_ADR + (eeprom_row * EE_OTOMATIK_ACMA_PARAM_SIZE), readData, sizeof(readData));

				uint16_t recete_num = DW_RECETE_ILK_ADR + combineBytes(readData[0], readData[1]);
				parse16BitTo8Bit(recete_num, &fakeRxBuffer[4], &fakeRxBuffer[5]);

				fakeRxBuffer[6] = 0x01;
				fakeRxBuffer[7] = 0x00;
				fakeRxBuffer[8] = 0x01;

				uint8_t crcBuffer[6];

				for(int i=3;i<9;i++)
					crcBuffer[i-3] = fakeRxBuffer[i];

				uint16_t crc = calculateCRC16Modbus(crcBuffer, sizeof(crcBuffer));

				fakeRxBuffer[9] = crc & 0xFF;
				fakeRxBuffer[10] = (crc >> 8) & 0xFF;

				for(int i=0;i<11;i++)
					DWIN_rxBuffer[i] = fakeRxBuffer[i];

				registerTable[REG_DW_MODE_INFO_ADR] = DW_RECETE_SAYFA_ENTER;

				DWIN.rxDoneFlag = 1;

			break;

		}



		otomatikPisirmeBaslatmaFlag = 0;
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

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	registerTable[DW_PISIRME_BASLATMA_ADR] 		= 0;
	registerTable[DW_BUHAR_HAZIRLAMA_ADR] 		= 0;
	registerTable[DW_BUHAR_PUSKURTME_ADR] 		= 0;
	registerTable[DW_TURBO_ADR] 				= 0;
	registerTable[DW_UST_SICAKLIK_ANIM] 		= 0;
	registerTable[DW_SURE_SONU_ALARM_ANIM_ADR] 	= 0;
	registerTable[DW_ALT_SICAKLIK_ANIM] 		= 0;
	registerTable[DW_ARIZA_PAGE_ADR]			= 0;
	registerTable[DW_BUHAR_HAZIR_ANIM]			= 0;
	registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR]	= 0;

	registerTable[REG_DW_MODE_INFO_ADR] = DW_ANA_SAYFA_ENTER;

	data = 0;

	DWIN_writeRegiser(&data, DW_PISIRME_SURESI_SN_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_PISIRME_BASLATMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_PISIRME_SONLANDIRMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_BUHAR_PUSKURTME_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_BUHAR_HAZIRLAMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));
	DWIN_writeRegiser(&data, DW_SURE_SONU_ALARM_ANIM_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_ARIZA_ALARM_SUSTURMA_ADR, sizeof(data));
	DWIN_writeRegiser(&data, DW_BUHAR_HAZIR_ANIM, sizeof(data));
}

void DWIN_arızaCheck(void)
{
	if((temp.TC1 >= 400) && (registerTable[DW_TC1_ARIZA_ADR] != 1) && (registerTable[DW_PARAM_BUHAR_SENSOR_TYPE_ADR] == DW_BUHAR_SENSOR_KUPL_VAL))
	{
		SEGGER_RTT_printf(0, "TC1 Arizasi ! \r\n");
		registerTable[DW_TC1_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_TC1_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_TC1_ARIZA_ADR, sizeof(data));
	}

	if(((registerTable[DW_PARAM_BUHAR_SENSOR_TYPE_ADR] == DW_BUHAR_SENSOR_TAT_VAL)||(temp.TC1 < 400)) && (registerTable[DW_TC1_ARIZA_ADR] != 0))
	{
		registerTable[DW_TC1_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_TC1_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_TC1_ARIZA_ADR, sizeof(data));
	}

	if((temp.TC2 >= 400) && (registerTable[DW_TC2_ARIZA_ADR] != 1) && (registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 1))
	{
		SEGGER_RTT_printf(0, "TC2 Arizasi ! \r\n");
		registerTable[DW_TC2_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_TC2_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_TC2_ARIZA_ADR, sizeof(data));
	}

	if(((registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)||(temp.TC2 < 400)) && (registerTable[DW_TC2_ARIZA_ADR] != 0))
	{
		registerTable[DW_TC2_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_TC2_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_TC2_ARIZA_ADR, sizeof(data));
	}

	if((temp.TC3 >= 400) && (registerTable[DW_TC3_ARIZA_ADR] != 1))
	{
		SEGGER_RTT_printf(0, "TC3 Arizasi ! \r\n");
		registerTable[DW_TC3_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_TC3_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_TC3_ARIZA_ADR, sizeof(data));
	}

	if((temp.TC3 < 400) && (registerTable[DW_TC3_ARIZA_ADR] != 0))
	{
		registerTable[DW_TC3_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_TC3_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_TC3_ARIZA_ADR, sizeof(data));
	}

	if((HAL_GPIO_ReadPin(I_KAPI_SWITCH) == 0)&&(registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] != 1))
	{
		SEGGER_RTT_printf(0, "Asiri Sicaklik Arizasi ! \r\n");
		registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] = 1;
		uint16_t data = 1;
		DWIN_writeRegiser(&data, DW_ASIRI_SICAKLIK_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_ASIRI_SICAKLIK_ARIZA_ADR, sizeof(data));
	}

	if((HAL_GPIO_ReadPin(I_KAPI_SWITCH) == 1)&&(registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] != 0))
	{
		registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] = 0;
		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_ASIRI_SICAKLIK_ARIZA_ADR, sizeof(data));
		STM32_RequestBufferWrite(&data, DW_ASIRI_SICAKLIK_ARIZA_ADR, sizeof(data));
	}

	if((	(registerTable[DW_TC1_ARIZA_ADR] == 1) ||
			(registerTable[DW_TC2_ARIZA_ADR] == 1) ||
			(registerTable[DW_TC3_ARIZA_ADR] == 1) ||
			(registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] == 1)) &&(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		registerTable[DW_ARIZA_PAGE_ADR] = 1;
		DWIN_changePage(DW_ARIZA_PAGE_ADR);
		setOut(K1|K2|K3|K4|K5|K6, 0);
		setOut(K10, 1);
		pisirmeSonuAlarmFlag = 1;
		alarmBuzzerPeriod = 200;
	}

	if((registerTable[DW_TC1_ARIZA_ADR] == 0) && (registerTable[DW_TC2_ARIZA_ADR] == 0) && (registerTable[DW_TC3_ARIZA_ADR] == 0) && (registerTable[DW_ASIRI_SICAKLIK_ARIZA_ADR] == 0) && (registerTable[DW_ARIZA_PAGE_ADR] != 0))
	{
		registerTable[DW_ARIZA_PAGE_ADR] = 0;
		registerTable[DW_ARIZA_ALARM_SUSTURMA_ADR] = 0;

		uint16_t data = 0;
		DWIN_writeRegiser(&data, DW_ARIZA_ALARM_SUSTURMA_ADR, sizeof(data));

		if(registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)
		{
			if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
			{
				if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
					DWIN_changePage(DW_PAGE_MANUEL_BUHARYOK_TEKTC_ADR);			// Manuel - tek tc buhar yok

				else
					DWIN_changePage(DW_PAGE_MANUEL_BUHARVAR_TEKTC_ADR);			// Manuel - tek tc buhar var
			}
			else
			{
				if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
					DWIN_changePage(DW_PAGE_MANUEL_BUHARYOK_CIFTTC_ADR);			// Manuel - cift tc buhar yok

				else
					DWIN_changePage(DW_PAGE_MANUEL_BUHARVAR_CIFTTC_ADR);			// Manuel - cift tc buhar var
			}
		}
		else if(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER)
		{
			if(registerTable[DW_PARAM_CIHAZ_TYPE_ADR] == 0)
			{
				if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
					DWIN_changePage(DW_PAGE_RECETE_BUHARYOK_TEKTC_ADR);			// Manuel - tek tc buhar yok

				else
					DWIN_changePage(DW_PAGE_RECETE_BUHARVAR_TEKTC_ADR);			// Manuel - tek tc buhar var
			}
			else
			{
				if(registerTable[DW_PARAM_BUHAR_ACTIVE_ADR] == 0)
					DWIN_changePage(DW_PAGE_RECETE_BUHARYOK_CIFTTC_ADR);			// Manuel - cift tc buhar yok

				else
					DWIN_changePage(DW_PAGE_RECETE_BUHARVAR_CIFTTC_ADR);			// Manuel - cift tc buhar var
			}
		}


		if(registerTable[DW_SURE_SONU_ALARM_ANIM_ADR] != 1)
		{

			pisirmeSonuAlarmFlag = 0;
			pisirmeSonuAlarmBuzzer = 0;
			setOut(BUZZER, 0);
			setOut(K10, 0);
		}
		alarmBuzzerPeriod = 1000;

		if(registerTable[DW_SURE_SONU_ALARM_ANIM_ADR] == 1)
		{
			data = 1;
			DWIN_writeRegiser(&data, DW_SURE_SONU_ALARM_ANIM_ADR, sizeof(data));
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

void PWM_SetFreqAndDuty(uint32_t freq_hz, uint8_t duty_percent)
{
    uint32_t timer_clk = HAL_RCC_GetPCLK2Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
        timer_clk *= 2;

    uint32_t prescaler = htim1.Init.Prescaler + 1;

    uint32_t arr = (timer_clk / (prescaler * freq_hz)) - 1;

    uint32_t ccr = ((arr + 1) * duty_percent) / 100;

    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr);
}


void PWM_StartSmoothTransition(uint32_t new_freq, uint8_t duty_percent) {
    start_freq = current_freq; // Şu an kaçtaysak oradan başla
    target_freq = new_freq;
    current_duty = duty_percent;
    tick_counter = 0;          // Sayacı sıfırla
    is_transitioning = 1;      // Süreci başlat
}

void PWM_SmoothTask_1ms(void) {
    if (is_transitioning) {
        tick_counter++;

        if (tick_counter <= 1000) {
            // Ara frekans hesaplama: start + (fark * tick / 1000)
            // Not: Çarpma işlemi uint32 sınırını aşmasın diye (int64_t) cast ediyoruz.
            // STM32 için bu işlem float'tan çok daha hızlıdır.
            int64_t delta = (int64_t)target_freq - (int64_t)start_freq;
            current_freq = (uint32_t)((int64_t)start_freq + (delta * tick_counter) / 1000);

            // PWM güncelleme
            PWM_SetFreqAndDuty(current_freq, current_duty);
        }

        if (tick_counter >= 1000) {
            // Garantiye al: Tam hedef değere eşitle ve dur
            current_freq = target_freq;
            PWM_SetFreqAndDuty(current_freq, current_duty);
            is_transitioning = 0;
            tick_counter = 0;
        }
    }
}

void setAnalogVoltage(float target_voltage, uint32_t Channel)
{

	// 2. Sabit Değerler
	const float V_REF = 3.3f;               // MCU besleme gerilimi (DAC referansı)
	float 		GAIN = 0; 					// Opamp Kazancı
	const float MAX_DAC_VAL = 4095.0f;      // 12-bit DAC maksimum değeri

	if(Channel == DAC_CHANNEL_1)
	{
		GAIN = 6.0;

		if (target_voltage > 15.0f) {
			target_voltage = 15.0f;
		} else if (target_voltage < 0.0f) {
			target_voltage = 0.0f;
		}
	}
	else if(Channel == DAC_CHANNEL_2)
	{
		GAIN = 3.26;

		if (target_voltage > 10.0f) {
			target_voltage = 10.0f;
		} else if (target_voltage < 0.0f) {
			target_voltage = 0.0f;
		}
	}

	// 3. İstenen 10V çıkış için, DAC'tan çıkması gereken hedef voltajı bul
	float required_dac_voltage = target_voltage / GAIN;

	// 4. Bu voltajı 0-4095 arasında bir dijital değere (RAW) çevir
	// Formül: (İstenen_DAC_Voltajı / V_REF) * 4095
	uint32_t dac_raw_value = (uint32_t)((required_dac_voltage / V_REF) * MAX_DAC_VAL);

	// 5. Değeri DAC kanalına yazdır (Kanal 1 kullanıldığını varsayıyoruz, Kanal 2 ise değiştirin)
	HAL_DAC_SetValue(&hdac, Channel, DAC_ALIGN_12B_R, dac_raw_value);
}

