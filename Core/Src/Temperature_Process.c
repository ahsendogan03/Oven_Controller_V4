/*
 * Temperature_Process.c
 *
 *  Created on: Jan 20, 2025
 *      Author: Step
 */

// Test Commit

#include "Temperature_Process.h"
#include "SEGGER_RTT.h"

//#include "DWIN_Process.h"
//#include "EEPROM_Process.h"
//#include "sht40.h"
//#include "hdc1080.h"
#include "TMP112.h"
#include "math.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

uint8_t adcInitFlag = 0;

uint32_t adcBuffer[numOfChannel];
uint32_t tempBuffer[numOfChannel];
uint32_t sumAdcBuffer[numOfChannel];
uint32_t avgAdcBuffer[numOfChannel];
uint32_t avgCounter = 0;

uint16_t vref;

TemperatureData temp;
//HDC1080 hdcSensor;
TMP112 tmpSensor;

extern uint8_t templog_free;
uint16_t templog_amount = 0;

//////////////////////////////////////////////////////////////////////////////////////////

int calculate_mcp9700(uint16_t adc_value);

/////////////////////////////////////////////////////////////////////////////////////////

int calculate_termocouple(uint16_t adc_value)
{
    float ref_adc_value = 500;    	// 23.5°C'deki referans ADC değeri
    float ref_temp = temp.TMP;    	// Referans sıcaklık (°C)
    float adc_coefficient; 			// ADC katsayısı (°C/LSB)

//    if(adc_value < 552)
//    	adc_coefficient = 4.325;

    if(adc_value < 620)
    	adc_coefficient = 5.80;

    else if((adc_value >= 620)&&(adc_value < 732))
    	adc_coefficient = 5.81;

    else if((adc_value >= 732)&&(adc_value < 848))
    	adc_coefficient = 5.85;

    else if((adc_value >= 848)&&(adc_value < 1025))
    	adc_coefficient = 5.87;

    else if((adc_value >= 1025)&&(adc_value < 1153))
    	adc_coefficient = 5.9;

    else if((adc_value >= 1153)&&(adc_value < 1334))
    	adc_coefficient = 5.93;

    else if((adc_value >= 1334)&&(adc_value < 1461))
    	adc_coefficient = 5.96;

    else if((adc_value >= 1461)&&(adc_value < 1649))
    	adc_coefficient = 5.98;

    else if((adc_value >= 1649)&&(adc_value < 1900))
    	adc_coefficient = 6.01;

    else if(adc_value >= 1900)
    	adc_coefficient = 6.07;

//    else if(adc_value >= 1849)
//    	adc_coefficient = 5.975;





    // 300 5.975
    // 260 5.935
    // 220 5.895
    // 190 5.825
    // 170 5.755
    // 140 5.685
    // 120 5.535
    // 90  5.365
    // 70  5.155
    // 50  4.825

    return (adc_value - ref_adc_value) / adc_coefficient + ref_temp;
}

int calculate_mcp9700(uint16_t adc_value) {
    // ADC değerinden voltajı hesapla
    float vout_mV = ((float)adc_value * vref) / 4095;

    // Sıcaklığı hesapla (Celsius)
    float temperature = (vout_mV - 500) / 10;

    return temperature;
}

uint16_t calculate_vref(uint16_t adc_vrefint) {
    // VREFINT için kalibrasyon değerini okuyoruz
    uint16_t vrefint_cal = VREFINT_CAL_VAL;

    // Gerçek referans voltajını hesaplıyoruz (mV cinsinden)
    float vref_actual = (float)vrefint_cal * VREF_CAL_VALUE / adc_vrefint;

    // mV'den Volt'a dönüştürüyoruz
    return vref_actual;
}


HAL_StatusTypeDef adc_Init(void)
{
	HAL_StatusTypeDef response = HAL_ERROR;

	if(HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK)
	{
		response = HAL_ADC_Start_DMA(&hadc1,adcBuffer,numOfChannel);
	}

	if(response == HAL_OK)
	{
		HAL_Delay(0);
		adcInitFlag = 1;
	}

	return response;

}

int ustOnSicaklik 		= 0;
int ustArkaSicaklik 	= 0;
int altSicaklik 		= 0;

void avgAdcProcess(void) // ms period function
{
	if(adcInitFlag)
	{

		for(int j=0;j<numOfChannel;j++)
		{
			sumAdcBuffer[j] += adcBuffer[j];
		}

		avgCounter++;

		if(avgCounter >= numOfSample)
		{
			for(int j=0;j<numOfChannel;j++)
			{
				avgAdcBuffer[j] = sumAdcBuffer[j] / numOfSample;
				sumAdcBuffer[j] = 0;
			}

			avgCounter = 0;
		}

	}
}

static const float K_TYPE_TABLE[] = {
    0.000,   0.397,   0.798,   1.203,   1.612,   // 0-40°C
    2.023,   2.436,   2.851,   3.267,   3.682,   // 50-90°C
    4.096,   4.509,   4.920,   5.328,   5.735,   // 100-140°C
    6.138,   6.540,   6.941,   7.340,   7.739,   // 150-190°C
    8.138,   8.539,   8.940,   9.343,   9.747,   // 200-240°C
    10.153,  10.561,  10.971,  11.382,  11.795,  // 250-290°C
    12.209,  12.624,  13.040,  13.457,  13.874,  // 300-340°C
    14.293,  14.713,  15.133,  15.554,  15.975,  // 350-390°C
    16.397,  16.820,  17.243,  17.667,  18.091,  // 400-440°C
    18.516,  18.941,  19.366,  19.792,  20.218,  // 450-490°C
    20.644,  21.071,  21.497,  21.924,  22.350,  // 500-540°C
    22.776,  23.203,  23.629,  24.055,  24.480,  // 550-590°C
    24.905,  25.330,  25.755,  26.179,  26.602,  // 600-640°C
    27.025,  27.447,  27.869,  28.289,  28.710,  // 650-690°C
    29.129,  29.548,  29.965,  30.382,  30.798,  // 700-740°C
    31.213,  31.628,  32.041,  32.453,  32.865,  // 750-790°C
    33.275,  33.685,  34.093,  34.501,  34.908,  // 800-840°C
    35.313,  35.718,  36.121,  36.524,  36.925,  // 850-890°C
    37.326,  37.725,  38.124,  38.522,  38.918,  // 900-940°C
    39.314,  39.708,  40.101,  40.494,  40.885,  // 950-990°C
    41.276,  41.665,  42.053,  42.440,  42.826,  // 1000-1040°C
    43.211,  43.595,  43.978,  44.359,  44.740,  // 1050-1090°C
    45.119,  45.497,  45.873,  46.249,  46.623,  // 1100-1140°C
    46.995,  47.367,  47.737,  48.105,  48.473,  // 1150-1190°C
    48.838,  49.202,  49.565,  49.926,  50.286,  // 1200-1240°C
    50.644,  51.000,  51.355,  51.708,  52.060,  // 1250-1290°C
    52.410,  52.759,  53.106,  53.451,  53.795,  // 1300-1340°C
    54.138                                        // 1370°C
};

// Tablodaki eleman sayısı
#define K_TABLE_SIZE (sizeof(K_TYPE_TABLE) / sizeof(K_TYPE_TABLE[0]))

// Sıcaklıktan voltaj hesaplama (lineer interpolasyon)
float K_Temp_to_mV(float temp_celsius) {
    if (temp_celsius < 0.0f) return 0.0f;
    if (temp_celsius >= 1370.0f) return K_TYPE_TABLE[K_TABLE_SIZE - 1];

    float index = temp_celsius / 10.0f;
    int lower_index = (int)index;
    int upper_index = lower_index + 1;

    if (upper_index >= K_TABLE_SIZE) {
        return K_TYPE_TABLE[K_TABLE_SIZE - 1];
    }

    float fraction = index - (float)lower_index;
    float lower_mv = K_TYPE_TABLE[lower_index];
    float upper_mv = K_TYPE_TABLE[upper_index];

    return lower_mv + (upper_mv - lower_mv) * fraction;
}

// Voltajdan sıcaklık hesaplama (lineer arama ve interpolasyon)
float K_mV_to_Temp(float voltage_mv) {
    if (voltage_mv <= 0.0f) return 0.0f;

    // Tabloda arama yap
    for (int i = 0; i < K_TABLE_SIZE - 1; i++) {
        if (voltage_mv >= K_TYPE_TABLE[i] && voltage_mv <= K_TYPE_TABLE[i + 1]) {
            // Lineer interpolasyon
            float temp_lower = (float)(i * 10);
            float temp_upper = (float)((i + 1) * 10);
            float mv_lower = K_TYPE_TABLE[i];
            float mv_upper = K_TYPE_TABLE[i + 1];

            float fraction = (voltage_mv - mv_lower) / (mv_upper - mv_lower);
            return temp_lower + (temp_upper - temp_lower) * fraction;
        }
    }

    return 1370.0f; // Maksimum sıcaklık
}

// K tipi termokupl için sıcaklık ölçüm fonksiyonu
int K_Thermocouple_GetTemp(uint16_t adc_value) {

    // Kalibrasyon verileri:
    // 75°C  → 730 ADC
    // 150°C → 1075 ADC

    const float ADC_1 = 730.0f;
    const float TEMP_1 = 75.0f;
    const float ADC_2 = 1075.0f;
    const float TEMP_2 = 150.0f;

    // Soğuk nokta kompanzasyon hatası düzeltmesi
    // Cihazınız 10°C fazla ölçüyor, bu soğuk nokta sensörünün
    // 10°C düşük okuduğu anlamına geliyor
    const float COLD_JUNCTION_OFFSET = 0.0f; // Düzeltme faktörü

    // Düzeltilmiş soğuk nokta sıcaklığı
    float corrected_cold_junction = temp.TMP + COLD_JUNCTION_OFFSET;

    // Bu sıcaklıklara karşılık gelen termokupl voltajları
    float mv_at_75C = K_Temp_to_mV(TEMP_1);
    float mv_at_150C = K_Temp_to_mV(TEMP_2);

    // Soğuk nokta (ortam sıcaklığı) voltajı
    float cold_junction_mv = K_Temp_to_mV(corrected_cold_junction);

    // 75°C ve 150°C ölçümlerinde termokupl üzerindeki net voltaj farkı
    float net_mv_75 = mv_at_75C - cold_junction_mv;
    float net_mv_150 = mv_at_150C - cold_junction_mv;

    // ADC'den voltaj hesaplama katsayısı
    float adc_to_mv_scale = (net_mv_150 - net_mv_75) / (ADC_2 - ADC_1);

    // Şu anki ADC değerinden net termokupl voltajını hesapla
    float current_net_mv = net_mv_75 + ((float)adc_value - ADC_1) * adc_to_mv_scale;

    // Soğuk nokta kompanzasyonu: toplam voltaj = net voltaj + soğuk nokta voltajı
    float total_mv = current_net_mv + cold_junction_mv;

    // Voltajdan sıcaklığa çevir
    float temperature = K_mV_to_Temp(total_mv);

    return (int)(temperature + 0.5f);
}

//75 790 100 890 150 1110

/* KALİBRASYON VERİLERİ
 * Referans 1: 75 Derece -> ADC 730
 * Referans 2: 150 Derece -> ADC 1075
 *
 * Hesaplanan Eğim (Slope): 4.6 ADC/Derece
 * Hesaplanan Offset (Sıfır Noktası): 500
 */
#define ADC_OFFSET_K      510
//#define ADC_PER_DEGREE  4.63f  //610 5.2 2100 4.65

float ConvertValueforKType(uint16_t input)
{
    if (input <= 550)
        return 5.10f;

    if (input >= 2100)
        return 4.69f;

    float t = (float)(input - 550) / (2100.0f - 550.0f); // 0..1

    // %10 daha agresif ease-out
    float curved = 1.0f - powf((1.0f - t), 2.2f);

    return 5.10f + curved * (4.69f - 5.10f);
}

int calculateThermocouple_K(uint16_t adc_value) {
    // 1. Adım: ADC değerinden sanal sıfır noktasını çıkar (Negatif olabilir)
    int32_t adc_difference = (int32_t)adc_value - ADC_OFFSET_K;

    float ADC_PER_DEGREE = ConvertValueforKType(adc_value);

    // 2. Adım: ADC farkını sıcaklık farkına çevir
    // float işlemi ile hassas bölme yapıp int'e çeviriyoruz.
    int32_t delta_temp = (int32_t)(adc_difference / ADC_PER_DEGREE);

    // 3. Adım: Ortam sıcaklığını (Cold Junction) ekle
    int result_temp = delta_temp + temp.TMP;

    return result_temp;
}

#define ADC_OFFSET_J      515

float ConvertValueforJType(uint16_t input)
{
    if (input <= 550)
        return 5.80f;     // artık düşükte küçük

    if (input >= 2500)
        return 6.30f;     // yüksekte büyük

    float t = (float)(input - 550) / (2500.0f - 550.0f); // 0..1

    float curved = 1.0f - powf((1.0f - t), 1.5f);

    return 5.80f + curved * (6.30f - 5.80f);
}

int calculateThermocouple_J(uint16_t adc_value) {
    // 1. Adım: ADC değerinden sanal sıfır noktasını çıkar (Negatif olabilir)
    int32_t adc_difference = (int32_t)adc_value - ADC_OFFSET_J;

    float ADC_PER_DEGREE = ConvertValueforJType(adc_value);

    // 2. Adım: ADC farkını sıcaklık farkına çevir
    // float işlemi ile hassas bölme yapıp int'e çeviriyoruz.
    int32_t delta_temp = (int32_t)(adc_difference / ADC_PER_DEGREE);

    // 3. Adım: Ortam sıcaklığını (Cold Junction) ekle
    int result_temp = delta_temp + temp.TMP;

    return result_temp;
}

uint8_t intTempSampleCounter = 0;
float tmp_temp;
void calculate_temperature(void)
{

	if(intTempSampleCounter == 0)
	{
		if(TMP112_ReadTemperature(&tmpSensor, &tmp_temp) == HAL_OK)
			temp.TMP = tmp_temp;
		else
			SEGGER_RTT_printf(0," TMP112 Read Temp ERROR ! \r\n");
	}

	intTempSampleCounter++;

	if(intTempSampleCounter >= 5)
		intTempSampleCounter = 0;

// 75 730 150 1075

	vref 			= calculate_vref(avgAdcBuffer[VREF_ROW_BUFFER]);
	//temp.MCP9700 	= calculate_mcp9700(avgAdcBuffer[MCP9700_ROW_BUFFER]);
	temp.TC1 		= calculate_termocouple(avgAdcBuffer[TC1_ROW_BUFFER]);

//	if(avgAdcBuffer[TC2_ROW_BUFFER] < 550) // TC2 fazla ölçüyor
		temp.TC2 	= calculateThermocouple_J((avgAdcBuffer[TC2_ROW_BUFFER]*1000)/992);
//	else
//		temp.TC2 	= calculate_termocouple((avgAdcBuffer[TC2_ROW_BUFFER]*1000)/1013);

//	temp.TC2 		= calculate_termocouple(avgAdcBuffer[TC2_ROW_BUFFER]);
	temp.TC3 		= calculateThermocouple_K(avgAdcBuffer[TC3_ROW_BUFFER]);
//	if(avgAdcBuffer[TC3_ROW_BUFFER] < 550) // TC2 fazla ölçüyor
//		temp.TC3 	= calculate_termocouple((avgAdcBuffer[TC3_ROW_BUFFER]*1000)/995);
//	else
//		temp.TC3 	= calculate_termocouple(avgAdcBuffer[TC3_ROW_BUFFER]);

//	if(avgAdcBuffer[TC3_ROW_BUFFER] < 550) // TC2 fazla ölçüyor
//		temp.TC3 	= calculate_termocouple((avgAdcBuffer[TC3_ROW_BUFFER]*1000)/1010);
//	else
//		temp.TC3 	= calculate_termocouple((avgAdcBuffer[TC3_ROW_BUFFER]*1000)/1013);


	ustOnSicaklik 	= temp.TC1;
	ustArkaSicaklik = temp.TC3;
	altSicaklik 	= temp.TC2;


	#if DEBUG_Temp == 1
	SEGGER_RTT_printf(0,"--------------------------------------------------------- \r\n");
	SEGGER_RTT_printf(0,"VREF_mv :  %d  TMP112 :     %d    TC1 Temp : %d   TC2 Temp : %d   TC3 Temp : %d \r\n",
						vref, temp.MCP9700, temp.TC1, temp.TC2, temp.TC3);

	SEGGER_RTT_printf(0,"VREF Val : %d  TMP112 : %d  TC1 Val  : %d  TC2 Val  : %d  TC3 Val  : %d \r\n",
						avgAdcBuffer[VREF_ROW_BUFFER],avgAdcBuffer[MCP9700_ROW_BUFFER],avgAdcBuffer[TC1_ROW_BUFFER],avgAdcBuffer[TC2_ROW_BUFFER],avgAdcBuffer[TC3_ROW_BUFFER]);

	#endif



}
