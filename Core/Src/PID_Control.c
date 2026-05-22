/*
 * PID_Control.c
 *
 *  Created on: Mar 19, 2025
 *      Author: Step
 */


#include "PID_Control.h"
#include "DWIN_Process.h"
#include "DWIN_Adress.h"
#include "Temperature_Process.h"
#include "SEGGER_RTT.h"
#include "EEPROM_Process.h"
#include "pid.h"
#include "InOut_Process.h"


uint16_t pwm_counter = 0; // Ortak sayaç (1000 ms için)

uint8_t pwm1_duty = 0;
uint8_t pwm2_duty = 0;

extern tickCounter counterTick;
extern I2C_HandleTypeDef hi2c1;


extern int 	ustOnSicaklik 	;
extern int	ustArkaSicaklik ;
extern int 	altSicaklik 	;

PID_TypeDef TPID_UstArka;
PID_TypeDef TPID_UstOn;
PID_TypeDef TPID_Alt;

double PIDOut_UstArka, TempSetpoint_UstArka;
double PIDOut_UstOn, TempSetpoint_UstOn;
double PIDOut_Alt, TempSetpoint_Alt;

extern uint16_t registerTable[9000];

double KP_ustArka = 0.3;
double KI_ustArka = 0.005;
double KD_ustArka = 0.0;

double KP_ustOn = 0.3;
double KI_ustOn = 0.005;
double KD_ustOn = 0.0;

double KP_alt = 0.5;
double KI_alt = 0.02;
double KD_alt = 0.0;

uint8_t PID_ustOnTurboCheck 	= 0;
uint8_t PID_ustArkaTurboCheck 	= 0;
uint8_t PID_AltTurboCheck 		= 0;

uint8_t TempSetpoint_UstArka_Arrive = 0;
uint8_t TempSetpoint_UstOn_Arrive 	= 0;
uint8_t TempSetpoint_Alt_Arrive 	= 0;

uint16_t ustArka_Turbo_Calc = 0;
uint16_t alt_Turbo_Calc 	= 0;

void PID_Setup(void)
{
	TempSetpoint_UstArka_Arrive = 0;
	TempSetpoint_UstOn_Arrive 	= 0;
	TempSetpoint_Alt_Arrive 	= 0;


	TempSetpoint_UstArka 	= registerTable[DW_UST_SICAKLIK_SET_ADR];
	TempSetpoint_UstOn	 	= registerTable[DW_UST_SICAKLIK_SET_ADR];
	TempSetpoint_Alt 		= registerTable[DW_ALT_SICAKLIK_SET_ADR];


	PID(&TPID_UstArka, &ustArkaSicaklik, &PIDOut_UstArka, &TempSetpoint_UstArka, KP_ustArka, KI_ustArka, KD_ustArka, _PID_P_ON_E, _PID_CD_DIRECT); // 0.8,0,0 //0.7, 0.001, 5.0//
	PID_SetMode(&TPID_UstArka, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID_UstArka, 1000);
	PID_SetOutputLimits(&TPID_UstArka, 0, 100);

	PID(&TPID_UstOn, &ustOnSicaklik, &PIDOut_UstOn, &TempSetpoint_UstOn, KP_ustOn, KI_ustOn, KD_ustOn, _PID_P_ON_E, _PID_CD_DIRECT); // 0.8,0,0 //0.7, 0.001, 5.0//
	PID_SetMode(&TPID_UstOn, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID_UstOn, 1000);
	PID_SetOutputLimits(&TPID_UstOn, 0, 100);

	PID(&TPID_Alt, &altSicaklik, &PIDOut_Alt, &TempSetpoint_Alt, KP_alt, KI_alt, KD_alt, _PID_P_ON_E, _PID_CD_DIRECT); // 0.8,0,0 //0.7, 0.001, 5.0//
	PID_SetMode(&TPID_Alt, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID_Alt, 1000);
	PID_SetOutputLimits(&TPID_Alt, 0, 100);

	if((TempSetpoint_UstArka - ustArkaSicaklik) > 0)
	{
		if((TempSetpoint_UstArka - ustArkaSicaklik) >= TEMP_APPROACH_VALUE)
		{
			ustArka_Turbo_Calc 	= (TEMP_APPROACH_VALUE/(TempSetpoint_UstArka - ustArkaSicaklik)) * TEMP_TURBO_LIMIT_DIFF_UST_ARKA;
		}
		else
		{
			ustArka_Turbo_Calc 	= (((TempSetpoint_UstArka - ustArkaSicaklik)/TEMP_APPROACH_VALUE) * TEMP_TURBO_LIMIT_DIFF_UST_ARKA)+((TempSetpoint_UstArka - ustArkaSicaklik)/10);

			if((TempSetpoint_UstArka - ustArkaSicaklik) <= 40)
			{
				ustArka_Turbo_Calc = ustArka_Turbo_Calc + (ustArka_Turbo_Calc/2);
			}
		}
	}
	else
	{
		ustArka_Turbo_Calc 	= 0;
	}

	if((TempSetpoint_Alt - altSicaklik) > 0)
	{
		if((TempSetpoint_Alt - altSicaklik) >= TEMP_APPROACH_VALUE)
		{
			alt_Turbo_Calc 	= (TEMP_APPROACH_VALUE/(TempSetpoint_Alt - altSicaklik)) * TEMP_TURBO_LIMIT_DIFF_ALT;
		}
		else
		{
			alt_Turbo_Calc 	= (((TempSetpoint_Alt - altSicaklik)/TEMP_APPROACH_VALUE) * TEMP_TURBO_LIMIT_DIFF_ALT)+((TempSetpoint_Alt - altSicaklik)/10);

			if((TempSetpoint_Alt - altSicaklik) <= 40)
			{
				alt_Turbo_Calc = alt_Turbo_Calc + (alt_Turbo_Calc/2);
			}
		}
	}
	else
	{
		alt_Turbo_Calc = 0;
	}


	SEGGER_RTT_printf(0,"ustArka_Turbo_Calc : %d \r\n",ustArka_Turbo_Calc);
	SEGGER_RTT_printf(0,"alt_Turbo_Calc : %d \r\n",alt_Turbo_Calc);


}

uint8_t dutyCycle_Calc_UstArka(uint16_t setTemp)
{
    int32_t duty = 60 + ((int32_t)setTemp - 250) * 20 / 50;

    // üst sınır 100 (if yok)
    duty = (duty > 100) * 100 + (duty <= 100) * duty;

    return (uint8_t)duty;
}

uint8_t dutyCycle_Calc_Alt(uint16_t setTemp)
{
    int32_t duty = 50 + ((int32_t)setTemp - 250) * 25 / 70;

    // sadece üst sınır clamp (if yok)
    duty = (duty > 100) * 100 + (duty <= 100) * duty;

    return (uint8_t)duty;
}

void PID_Run()
{
	uint16_t dutyCycle_ustArka 	= 0;
	uint16_t dutyCycle_Alt		= 0;

	PID_Turbo_Check();
	TempSetPoint_Arrive_Check();


	if((PID_ustArkaTurboCheck == 1)&&(TempSetpoint_UstArka_Arrive != 1))
	{
		PIDOut_UstArka = 0;
		TPID_UstArka.OutputSum = 0;

		dutyCycle_ustArka = 100;

		SEGGER_RTT_printf(0,"Ust Arka A\r\n");
	}

	else if(TempSetpoint_UstArka_Arrive != 1)
	{
		if((TempSetpoint_UstArka - (ustArka_Turbo_Calc/5)) > ustArkaSicaklik)
		{
			PIDOut_UstArka = 0;
			TPID_UstArka.OutputSum = 0;

			dutyCycle_ustArka = dutyCycle_Calc_UstArka(TempSetpoint_UstArka);

			SEGGER_RTT_printf(0,"Ust Arka B\r\n");
		}
		else if((TempSetpoint_UstArka - 1) > ustArkaSicaklik)
		{
			TPID_UstArka.Ki = 0.02;

			if(TempSetpoint_UstArka >= 300)
				TPID_UstArka.Ki = 0.2;

			PID_Compute(&TPID_UstArka,HAL_GetTick());
			dutyCycle_ustArka = PIDOut_UstArka;

			SEGGER_RTT_printf(0,"Ust Arka C\r\n");
		}

		else
		{
			PID_Compute(&TPID_UstArka,HAL_GetTick());

			dutyCycle_ustArka = 0;
			SEGGER_RTT_printf(0,"Ust Arka G\r\n");
		}
	}

	else
	{
		if((TempSetpoint_UstArka - 7) > ustArkaSicaklik)
		{
			PIDOut_UstArka = 0;
			TPID_UstArka.OutputSum = 0;

			dutyCycle_ustArka = 100;

			SEGGER_RTT_printf(0,"Ust Arka D\r\n");
		}
		else if(ustArkaSicaklik < (TempSetpoint_UstArka ))
		{
			if(TempSetpoint_UstArka >= 300)
				TPID_UstArka.Ki = 0.2;

			PID_Compute(&TPID_UstArka,HAL_GetTick());
			dutyCycle_ustArka = PIDOut_UstArka;

			SEGGER_RTT_printf(0,"Ust Arka E\r\n");

		}
		else if(ustArkaSicaklik >=  (TempSetpoint_UstArka))
		{
			PID_Compute(&TPID_UstArka,HAL_GetTick());

			dutyCycle_ustArka = 0;
			SEGGER_RTT_printf(0,"Ust Arka F\r\n");
		}
	}

	if((PID_AltTurboCheck == 1)&&(TempSetpoint_Alt_Arrive != 1))
	{
		PIDOut_Alt = 0;
		TPID_Alt.OutputSum = 0;

		dutyCycle_Alt = 100;

		SEGGER_RTT_printf(0,"Alt A\r\n");
	}

	else if(TempSetpoint_Alt_Arrive != 1)
	{

		if((TempSetpoint_Alt - (alt_Turbo_Calc/4)) >= altSicaklik)
		{
			dutyCycle_Alt = dutyCycle_Calc_Alt(TempSetpoint_Alt);

			SEGGER_RTT_printf(0,"Alt B\r\n");
		}
		else
		{
			if(TempSetpoint_Alt >= 300 )
				TPID_Alt.Ki = 0.3;

			PID_Compute(&TPID_Alt,HAL_GetTick());
			dutyCycle_Alt = PIDOut_Alt;
			SEGGER_RTT_printf(0,"Alt C\r\n");
		}

	}

	else
	{

		if((TempSetpoint_Alt - 3) >= altSicaklik)
		{
			PIDOut_Alt = 0;
			TPID_Alt.OutputSum = 0;
			dutyCycle_Alt = 100;
			SEGGER_RTT_printf(0,"Alt D\r\n");

		}
		else if(altSicaklik < TempSetpoint_Alt)
		{
			TPID_Alt.Ki = 0.1;

			if(TempSetpoint_Alt >= 300 )
				TPID_Alt.Ki = 0.3;

			SEGGER_RTT_printf(0,"Alt E\r\n");
			PID_Compute(&TPID_Alt,HAL_GetTick());
			dutyCycle_Alt = PIDOut_Alt;
		}

		else if(altSicaklik >= (TempSetpoint_Alt))
		{
			PIDOut_Alt = 0;
			TPID_Alt.OutputSum = 0;
			SEGGER_RTT_printf(0,"Alt F\r\n");
			dutyCycle_Alt = 0;
		}

	}


	//SEGGER_RTT_printf(0, "Set Temp : %d  Temperature: %d  PID Value: %d Duty Cycle : %d last time : %d \r\n",(int)TempSetpoint, (int)Temp, (int16_t)PIDOut,dutyCycle,(int)TPID.LastTime);


	pwm2_duty 	= (dutyCycle_ustArka);
	pwm1_duty 	= (dutyCycle_Alt);

	SEGGER_RTT_printf(0,"------------------------------------------------------------------------------------------------ \r\n");
	SEGGER_RTT_printf(0,"Temp Ust Arka:  %d  Temp Alt:%d   -- UST SET : %d -- ALT SET : %d --  \r\n ",ustArkaSicaklik,altSicaklik,(int)TempSetpoint_UstOn,(int)TempSetpoint_Alt);
	SEGGER_RTT_printf(0,"PID Ust Arka :  %d  PID Alt :%d \r\n",pwm2_duty,pwm1_duty);

	PID_animationProcess();

}

void PID_Turbo_Check(void)
{

	if((PID_ustArkaTurboCheck != 1) && ((ustArkaSicaklik) < (TempSetpoint_UstArka - ustArka_Turbo_Calc)))
	{
		PID_ustArkaTurboCheck = 1;
	}

	if((PID_ustArkaTurboCheck != 0) && (ustArkaSicaklik >= (TempSetpoint_UstArka - ustArka_Turbo_Calc)))
	{
		PID_ustArkaTurboCheck = 0;
		TempSetpoint_UstArka_Arrive = 0;
	}

	if((PID_AltTurboCheck != 1) && ((altSicaklik) < (TempSetpoint_Alt - alt_Turbo_Calc)))
	{
		PID_AltTurboCheck = 1;
	}

	if((PID_AltTurboCheck != 0) && ((altSicaklik) >= (TempSetpoint_Alt - alt_Turbo_Calc)))
	{
		PID_AltTurboCheck = 0;
		TempSetpoint_Alt_Arrive = 0;
	}

}

void TempSetPoint_Arrive_Check(void)
{
	if((ustArkaSicaklik >= TempSetpoint_UstArka) && (TempSetpoint_UstArka_Arrive == 0))
		TempSetpoint_UstArka_Arrive = 1;

	if((ustOnSicaklik >= TempSetpoint_UstOn) && (TempSetpoint_UstOn_Arrive == 0))
		TempSetpoint_UstOn_Arrive = 1;

	if((altSicaklik >= TempSetpoint_Alt) && (TempSetpoint_Alt_Arrive == 0))
		TempSetpoint_Alt_Arrive = 1;
}

void manual_pwm_update(void)
{
	if(((registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)||(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER))&&
		(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		// PWM1
		if (pwm_counter < (pwm1_duty * 10))
			setOutData(K2, 1);
		else
			setOutData(K2, 0);


		// PWM2
		if (pwm_counter < (pwm2_duty * 10))
			setOutData(K4, 1);
		else
			setOutData(K4, 0);


		// Sayaç güncelleme
		pwm_counter++;
		if (pwm_counter >= 1000) {
			pwm_counter = 0;
		}
	}
}

void pwmOutProcess(void)
{
	if(((registerTable[REG_DW_MODE_INFO_ADR] == DW_MANUEL_MODE_ENTER)||(registerTable[REG_DW_MODE_INFO_ADR] == DW_RECETE_PISIRME_SAYFA_ENTER))&&
		(registerTable[DW_ARIZA_PAGE_ADR] != 1))
	{
		shiftRefresh();
	}
}

void PID_animationProcess(void)
{
	if(registerTable[DW_TURBO_ADR] == 1)
	{
		if((pwm1_duty != 100)&&(pwm2_duty != 100))
		{
			uint16_t data = 0;

			registerTable[DW_TURBO_ADR] = data;
			DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));
		}
	}
	else
	{
		if((pwm1_duty == 100)||(pwm2_duty == 100))
		{
			uint16_t data = 1;

			registerTable[DW_TURBO_ADR] = data;
			DWIN_writeRegiser(&data, DW_TURBO_ADR, sizeof(data));
		}
	}

	if((pwm1_duty > 0)&&(registerTable[DW_ALT_SICAKLIK_ANIM] != 1))
	{
		uint16_t data = 1;

		registerTable[DW_ALT_SICAKLIK_ANIM] = data;
		DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));

	}
	if((pwm1_duty == 0)&&(registerTable[DW_ALT_SICAKLIK_ANIM] == 1))
	{
		uint16_t data = 0;

		registerTable[DW_ALT_SICAKLIK_ANIM] = data;
		DWIN_writeRegiser(&data, DW_ALT_SICAKLIK_ANIM, sizeof(data));

	}

	if((pwm2_duty > 0)&&(registerTable[DW_UST_SICAKLIK_ANIM] != 1))
	{
		uint16_t data = 1;

		registerTable[DW_UST_SICAKLIK_ANIM] = data;
		DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));

	}
	if((pwm2_duty == 0)&&(registerTable[DW_UST_SICAKLIK_ANIM] == 1))
	{
		uint16_t data = 0;

		registerTable[DW_UST_SICAKLIK_ANIM] = data;
		DWIN_writeRegiser(&data, DW_UST_SICAKLIK_ANIM, sizeof(data));

	}
}


