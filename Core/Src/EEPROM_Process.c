/*
 * EEPROM_Process.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Step
 */


#include "EEPROM_Process.h"
#include "string.h"
#include "SEGGER_RTT.h"
#include "DWIN_Process.h"
#include "Temperature_Process.h"

EEPROM_initResponse eepromStatus = EE_INIT_OK;

uint8_t templog_free = 0;

uint16_t eepromAddrTable[14] = {	DW_UST_SICAKLIK_SET_ADR,
								DW_ALT_SICAKLIK_SET_ADR,
								DW_UST_ON_SET_ADR,
								DW_UST_ARKA_SET_ADR,
								DW_ALT_SET_ADR,
								DW_PISIRME_SURESI_ADR,
								DW_BUHAR_SURESI_ADR,
								DW_LAMBA_SURESI_ADR,
								DW_UST_ON_ISITICI_BANDI_ADR,
								DW_UST_ARKA_ISITICI_BANDI_ADR,
								DW_ALT_ISITICI_BANDI_ADR,
								DW_ISITICI_UST_HIS_ADR,
								DW_ISITICI_ALT_HIS_ADR,
								DW_ISITICI_PERIOD_ADR,
							};

extern uint16_t registerTable[9000];
extern uint16_t receteAdimSayisiTable[DW_RECETE_AMOUNT];

EEPROM_writeResponse AT24C256_WaitForReady(I2C_HandleTypeDef *hi2c) {

	EEPROM_writeResponse response = EEPROM_BUSY;
	uint16_t timeOut = 0;

	while ((response == EEPROM_BUSY)&&(timeOut <= EEPROM_TIMEOUT_MS))
	{
		if(HAL_I2C_IsDeviceReady(hi2c, EEPROM_BASE_ADDR << 1 , 1, HAL_MAX_DELAY) == HAL_OK)
			response = EE_WRITE_OK;

		timeOut++;
		HAL_Delay(0);
	}

    return response;
}

EEPROM_writeResponse EEPROM_Write(I2C_HandleTypeDef *hi2c,uint16_t memAddress, uint8_t *data, uint16_t dataSize)
{
	EEPROM_writeResponse response = EE_WRITE_OK;

	if(eepromStatus == EE_INIT_OK)
	{

		uint8_t i2c_addr = (EEPROM_BASE_ADDR << 1) | 0;  // R/W = 0 (Yazma)

		uint16_t remainingBytes = dataSize;  	// Kalan veri miktarı
		uint16_t currentAddress = memAddress;  	// Başlangıç adresi
		uint8_t *currentData 	= data;  		// Yazılacak veri pointer'ı

		uint8_t attempt = 0;

		whileEnterPoint:

		while ((remainingBytes > 0)&&(response == EE_WRITE_OK))
		{

			// Sayfa sınırını kontrol ederek maksimum yazılabilecek boyutu belirle
			uint16_t pageOffset = currentAddress % EEPROM_PAGE_SIZE;
			uint16_t writeSize = EEPROM_PAGE_SIZE - pageOffset;

			// Eğer kalan veri daha küçükse kalan veriyi yaz
			if (remainingBytes < writeSize) {
				writeSize = remainingBytes;
			}

			uint8_t buffer[2 + EEPROM_PAGE_SIZE];  					// Adres + veri
			buffer[0] = (uint8_t)((currentAddress >> 8) & 0xFF); 	// Yüksek byte adres
			buffer[1] = (uint8_t)(currentAddress & 0xFF);        	// Düşük byte adres

			memcpy(&buffer[2], currentData, writeSize);

			// Veri blogunu EEPROM'e yaz

			if (HAL_I2C_Master_Transmit(hi2c, i2c_addr, buffer, writeSize + 2, HAL_MAX_DELAY) != HAL_OK)
				response = EE_WRITE_ERROR;  // Hata durumunda işlemi kes


			// EEPROM yazmasi icin gereken sure
			if(AT24C256_WaitForReady(hi2c) != EE_WRITE_OK)
			{
				SEGGER_RTT_printf(0,"EEPROM TIMEOUT ! \r\n");
				response = EE_WRITE_ERROR;
				goto whileEnterPoint;
			}
			// Yazılan miktarı güncelle
			remainingBytes -= writeSize;
			currentAddress += writeSize;
			currentData += writeSize;
		}

		if(response == EE_WRITE_OK)
		{

			uint8_t readData[dataSize];
			uint8_t check = 0;

			EEPROM_Read(hi2c, memAddress, readData, dataSize);

			for(int i=0;i<dataSize;i++)
			{

				if(readData[i] != data[i])
				{
					check = 1;
					break;
				}

			}

			if(check == 1)
			{
				attempt++;

				if(attempt <= 3)
					goto whileEnterPoint;

				SEGGER_RTT_printf(0,"EEPROM WRITE ERROR ! \r\n");
				eepromStatus = EE_INIT_ERROR;
				response = EE_WRITE_ERROR;

			}
		}

		else
		{
			attempt++;

			if(attempt <= 3)
			{
				response = EE_WRITE_OK;
				goto whileEnterPoint;
			}

			eepromStatus = EE_INIT_ERROR;
		}
	}

	else
		response = EE_WRITE_ERROR;


    return response;
}


void EEPROM_Read(I2C_HandleTypeDef *hi2c, uint16_t mem_address, uint8_t *data, uint16_t len) {

    uint8_t i2c_addr = (EEPROM_BASE_ADDR << 1) | 0;  // R/W = 0 (Yazma modunda adres belirtmek için)

    uint8_t address[2];
    address[0] = (mem_address >> 8) & 0xFF;  // Adresin üst 8 biti
    address[1] = mem_address & 0xFF;         // Adresin alt 8 biti

    // EEPROM'dan veri okumadan önce okuma adresini gönder
    if (HAL_I2C_Master_Transmit(hi2c, i2c_addr, address, 2, HAL_MAX_DELAY) != HAL_OK) {
        // Adres gönderimi sırasında hata oluştu
        return;
    }

    // Okuma işlemi için R/W = 1 ile veri oku
    i2c_addr = (EEPROM_BASE_ADDR << 1) | 1;  // R/W = 1 (Okuma)
    if (HAL_I2C_Master_Receive(hi2c, i2c_addr, data, len, HAL_MAX_DELAY) != HAL_OK) {
        // Okuma sırasında hata oluştu
        return;
    }
}

HAL_StatusTypeDef EEPROM_Read_Safe(I2C_HandleTypeDef *hi2c, uint16_t mem_address, uint8_t *data, uint16_t total_len) {
    if ((uint32_t)mem_address + total_len > EEPROM_PAGE_LIMIT) {
        // EEPROM sınırını aşacak bir okuma talebi var
        return HAL_ERROR;
    }

    uint16_t remaining = total_len;
    uint16_t offset = 0;

    while (remaining > 0) {
        uint16_t chunk = EEPROM_MAX_READ_CHUNK;

        // Sayfa sonunu geçmeyi engelle
        uint16_t bytes_to_end_of_page = EEPROM_PAGE_LIMIT - (mem_address + offset);
        if (chunk > bytes_to_end_of_page)
            chunk = bytes_to_end_of_page;

        if (chunk > remaining)
            chunk = remaining;

        // EEPROM_Read() senin verdiğin fonksiyon olmalı, 2-byte adresli
        EEPROM_Read(hi2c, mem_address + offset, data + offset, chunk);

        offset += chunk;
        remaining -= chunk;
    }

    return HAL_OK;
}

EEPROM_initResponse EEPROM_init(I2C_HandleTypeDef *hi2c)
{
	EEPROM_initResponse response = EE_INIT_OK;

	uint8_t usageCheck = 0;

//	uint8_t eraseWrite = 0;
//	EEPROM_Write(hi2c, EEPROM_USAGE_CHECK_ADDR, &eraseWrite, 1);
//	HAL_Delay(0);

	EEPROM_Read(hi2c, EEPROM_USAGE_CHECK_ADDR, &usageCheck, sizeof(usageCheck));


	if(usageCheck == EEPROM_USAGE_CHECK_VAL)
	{

		for(int i=0;i<sizeof(eepromAddrTable)/2;i++)
		{
			uint8_t readData[2];

			EEPROM_Read(hi2c, eepromAddrTable[i], readData, sizeof(readData));

			registerTable[eepromAddrTable[i]] = combineBytes(readData[0], readData[1]);

		}

		EEPROM_Recete_Read(hi2c);
		EEPROM_OtomatikAcma_Read(hi2c);

		registerTable[DW_PISIRME_SURESI_ORT_ADR] 	= registerTable[DW_PISIRME_SURESI_ADR];
		registerTable[DW_BUHAR_SURESI_ORT_ADR] 		= registerTable[DW_BUHAR_SURESI_ADR];

	}

	else
	{
		EEPROM_writeResponse check;

		registerTable[DW_UST_SICAKLIK_SET_ADR] 		= 200;
		registerTable[DW_ALT_SICAKLIK_SET_ADR] 		= 200;
		registerTable[DW_UST_ON_SET_ADR]			= 30;
		registerTable[DW_UST_ARKA_SET_ADR] 			= 20;
		registerTable[DW_ALT_SET_ADR] 				= 10;
		registerTable[DW_PISIRME_SURESI_ADR]		= 40;
		registerTable[DW_PISIRME_SURESI_ORT_ADR] 	= registerTable[DW_PISIRME_SURESI_ADR];
		registerTable[DW_BUHAR_SURESI_ADR] 			= 30;
		registerTable[DW_BUHAR_SURESI_ORT_ADR] 		= registerTable[DW_BUHAR_SURESI_ADR];
		registerTable[DW_LAMBA_SURESI_ADR]			= 60;
		registerTable[DW_UST_ON_ISITICI_BANDI_ADR]	= 50;
		registerTable[DW_UST_ARKA_ISITICI_BANDI_ADR]= 50;
		registerTable[DW_ALT_ISITICI_BANDI_ADR]		= 50;
		registerTable[DW_ISITICI_UST_HIS_ADR]		= 0;
		registerTable[DW_ISITICI_ALT_HIS_ADR]		= 0;
		registerTable[DW_ISITICI_PERIOD_ADR]		= 60;

		uint8_t usageWrite = EEPROM_USAGE_CHECK_VAL;
		EEPROM_Write(hi2c, EEPROM_USAGE_CHECK_ADDR, &usageWrite, 1);

		for(int i=0;i<sizeof(eepromAddrTable)/2;i++)
		{
			uint8_t writeData[2];

			parse16BitTo8Bit(registerTable[eepromAddrTable[i]], &writeData[0], &writeData[1]);

			check = EEPROM_Write(hi2c, eepromAddrTable[i], writeData, sizeof(writeData));

			if(check != EE_WRITE_OK)
			{
				response = EE_INIT_ERROR;
				break;
			}

		}

		if(response == EE_INIT_OK)
			response = EEPROM_Recete_DefaultWrite(hi2c);

		HAL_Delay(0);

		if(response == EE_INIT_OK)
			response = EEPROM_OtomatikAcma_DefaultWrite(hi2c);

		HAL_Delay(0);

		if(response == EE_INIT_OK)
			EEPROM_Recete_Read(hi2c);

		HAL_Delay(0);

		if(response == EE_INIT_OK)
			EEPROM_OtomatikAcma_Read(hi2c);


	}

	return response;
}
EEPROM_initResponse EEPROM_Recete_DefaultWrite(I2C_HandleTypeDef *hi2c)
{
	EEPROM_initResponse response = EE_INIT_OK;

	EEPROM_writeResponse check;

	uint8_t defaultRecete_isim[DW_RECETE_ISIM_SIZE] 		= {'N','o',' ','N','a','m','e',0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint16_t defaultRecete_data[EE_RECETE_DATA_SIZE/2] 		= {	200, 200, 10, 20, 30, 5, 25,
																200, 200, 10, 20, 30, 6, 25,
																200, 200, 10, 20, 30, 7, 25,
																200, 200, 10, 20, 30, 8, 25,
																0, 	 3
																};

	uint8_t defaultRecete_data1_u8[EE_RECETE_DATA_SIZE];
	uint8_t defaultRecete_data2_u8[EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE];

	convert_u16_to_u8(defaultRecete_data, defaultRecete_data1_u8, sizeof(defaultRecete_data));

	for(int i=0;i<EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE;i++)
	{
		if(i<EE_RECETE_ISIM_ROW)
			defaultRecete_data2_u8[i] = defaultRecete_data1_u8[i];
		else if(i<76)
			defaultRecete_data2_u8[i] = defaultRecete_isim[i - EE_RECETE_ISIM_ROW];
		else
			defaultRecete_data2_u8[i] = defaultRecete_data1_u8[i - 20];
	}

	uint8_t defaultRecete_allData_u8[(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)*DW_RECETE_AMOUNT];

	for(int i=0;i<DW_RECETE_AMOUNT;i++)
	{
		for(int j=0;j<EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE;j++)
		{
			defaultRecete_allData_u8[j+(i*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE))] = defaultRecete_data2_u8[j];
		}
	}

	check = EEPROM_Write(hi2c, EE_RECETE_ILK_ADR, defaultRecete_allData_u8, sizeof(defaultRecete_allData_u8));

	if(check != EE_WRITE_OK)
		response = EE_INIT_ERROR;

	return response;

}

void EEPROM_Recete_Read(I2C_HandleTypeDef *hi2c)
{
	for(int i=0;i<DW_RECETE_AMOUNT;i++)
	{
		uint8_t recete_isim_data_u8[DW_RECETE_ISIM_SIZE];
		EEPROM_Read_Safe(hi2c, EE_RECETE_ILK_ADR + EE_RECETE_ISIM_ROW +(i*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_isim_data_u8, sizeof(recete_isim_data_u8));

		uint16_t recete_isim_data_u16[DW_RECETE_ISIM_SIZE/2];

		for(int j=0;j<DW_RECETE_ISIM_SIZE/2;j++)
		{
			recete_isim_data_u16[j] = combineBytes(recete_isim_data_u8[j*2], recete_isim_data_u8[(j*2)+1]);
		}

		DWIN_writeRegiser(recete_isim_data_u16, DW_RECETE_ISIM_ILK_ADR + (i*(DW_RECETE_ISIM_SIZE/2)), sizeof(recete_isim_data_u16));

		uint8_t recete_resim_data_u8[2];
		uint16_t recete_resim_data_16;

		EEPROM_Read_Safe(hi2c, EE_RECETE_ILK_ADR + 76 + (i*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), recete_resim_data_u8, sizeof(recete_resim_data_u8));
		recete_resim_data_16 = combineBytes(recete_resim_data_u8[0], recete_resim_data_u8[1]);

		DWIN_writeRegiser(&recete_resim_data_16, DW_RECETE_RESIM_ILK_ADR + i, sizeof(recete_resim_data_16));
	}
}

EEPROM_initResponse EEPROM_OtomatikAcma_DefaultWrite(I2C_HandleTypeDef *hi2c)
{


	EEPROM_initResponse response = EE_INIT_OK;

	EEPROM_writeResponse check;

	//uint8_t defaultOtomatik_isim[DW_RECETE_ISIM_SIZE] 				= {'N','o',' ','N','a','m','e',0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint16_t defaultOtomatik_param[EE_OTOMATIK_ACMA_PARAM_SIZE/2] 	= {6,30,200,200,1,1,0,0}; // Saat, dakika, ust, alt, buhar, manuel/recete, aktif , recete num
	uint8_t defaultOtomatik_param_u8[EE_OTOMATIK_ACMA_PARAM_SIZE]	= {0};


	convert_u16_to_u8(defaultOtomatik_param, defaultOtomatik_param_u8, sizeof(defaultOtomatik_param_u8));


	uint8_t defaultOtomatik_allData[sizeof(defaultOtomatik_param_u8) * 7] = {0};

	for(int i=0;i<7;i++)
	{
		for(int j=0;j<sizeof(defaultOtomatik_param_u8);j++)
			defaultOtomatik_allData[j + (i*sizeof(defaultOtomatik_param_u8))] = defaultOtomatik_param_u8[j];
	}

	check = EEPROM_Write(hi2c, EE_OTOMATIK_ACMA_ILK_ADR, defaultOtomatik_allData, sizeof(defaultOtomatik_allData));

	if(check != EE_WRITE_OK)
		response = EE_INIT_ERROR;

	return response;

}

void EEPROM_OtomatikAcma_Read(I2C_HandleTypeDef *hi2c)
{
	uint8_t defaultOtomatik_allData[(EE_OTOMATIK_ACMA_PARAM_SIZE) * 7] = {0};
	EEPROM_Read_Safe(hi2c, EE_OTOMATIK_ACMA_ILK_ADR, defaultOtomatik_allData, sizeof(defaultOtomatik_allData));

	uint8_t defaultOtomatik_param_u8[EE_OTOMATIK_ACMA_PARAM_SIZE]		= {0};
	uint8_t defaultOtomatik_isim[DW_RECETE_ISIM_SIZE]					= {0};

	uint16_t defaultOtomatik_param_u16[EE_OTOMATIK_ACMA_PARAM_SIZE/2]	= {0};
	uint16_t defaultOtomatik_isim_u16[DW_RECETE_ISIM_SIZE/2]			= {0};

	for(int i=0;i<7;i++)
	{
		for(int j=0;j<EE_OTOMATIK_ACMA_PARAM_SIZE;j++)
			defaultOtomatik_param_u8[j] = defaultOtomatik_allData[j+(i*(EE_OTOMATIK_ACMA_PARAM_SIZE))];

		convert_u8_to_u16(defaultOtomatik_param_u8, defaultOtomatik_param_u16, sizeof(defaultOtomatik_param_u8));

		uint16_t otomatik_recete_row = defaultOtomatik_param_u16[7];

		EEPROM_Read_Safe(hi2c, EE_RECETE_ILK_ADR + EE_RECETE_ISIM_ROW +((otomatik_recete_row)*(EE_RECETE_DATA_SIZE + DW_RECETE_ISIM_SIZE)), defaultOtomatik_isim, sizeof(defaultOtomatik_isim));

		convert_u8_to_u16(defaultOtomatik_isim, defaultOtomatik_isim_u16, sizeof(defaultOtomatik_isim));

		for(int k=0;k<(EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3;k++)
			registerTable[(DW_OTOMATIK_ACMA_ILK_ADR + k) + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] =  defaultOtomatik_param_u16[k];

		registerTable[DW_OTOMATIK_PISIRME_INFO_ADR + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] 	= defaultOtomatik_param_u16[(EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3];
		registerTable[DW_OTOMATIK_AKTIF_INFO_ADR + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] 	= defaultOtomatik_param_u16[(EE_OTOMATIK_ACMA_PARAM_SIZE/2)-2];

		for(int k=0;k<DW_RECETE_ISIM_SIZE/2;k++)
			registerTable[(DW_OTOMATIK_ISIM_ILK_ADR + k) + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] = defaultOtomatik_isim_u16[k];


		uint16_t writeData[((EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3) + (DW_RECETE_ISIM_SIZE / 2)] = {0};

		for(int k=0;k<sizeof(writeData)/2;k++)
		{
			if(k<((EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3))
				writeData[k] = defaultOtomatik_param_u16[k];
			else
				writeData[k] = defaultOtomatik_isim_u16[k - ((EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3)];
		}

		DWIN_writeRegiser(writeData, DW_OTOMATIK_ACMA_ILK_ADR + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(writeData));

		if(defaultOtomatik_param_u16[(EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3] == 1)
		{
			uint16_t oto_write = 0x0101;
			DWIN_writeRegiser(&oto_write, 0x15A2 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));

			oto_write = 0x1812;
			DWIN_writeRegiser(&oto_write, 0x15A9 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));
			DWIN_writeRegiser(&oto_write, 0x15B6 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));

			oto_write = 0x0035;
			DWIN_writeRegiser(&oto_write, 0x1595 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));
		}
		else if(defaultOtomatik_param_u16[(EE_OTOMATIK_ACMA_PARAM_SIZE/2)-3] == 2)
		{
			uint16_t oto_write = 0x162C;
			DWIN_writeRegiser(&oto_write, 0x15A2 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));

			oto_write = 0x0101;
			DWIN_writeRegiser(&oto_write, 0x15A9 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));
			DWIN_writeRegiser(&oto_write, 0x15B6 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));

			oto_write = 0x003A;
			DWIN_writeRegiser(&oto_write, 0x1595 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));
		}
		if(defaultOtomatik_param_u16[(EE_OTOMATIK_ACMA_PARAM_SIZE/2)-2] == 0)
		{
			uint16_t oto_write = 0x003D;
			DWIN_writeRegiser(&oto_write, 0x1597 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));
			registerTable[0x1597 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] = 0;
		}
		else
		{
			if(registerTable[DW_SAAT_IKON_ADDR] != 1)
			{
				uint16_t ikonWrite = 1;
				DWIN_writeRegiser(&ikonWrite, DW_SAAT_IKON_ADDR, sizeof(ikonWrite));
				registerTable[DW_SAAT_IKON_ADDR] = 1;
			}
			uint16_t oto_write = 0x003E;
			DWIN_writeRegiser(&oto_write, 0x1597 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH), sizeof(oto_write));
			registerTable[0x1597 + (i*DW_OTOMATIK_ACMA_ADR_LENGTH)] = 1;
		}


	}

}
