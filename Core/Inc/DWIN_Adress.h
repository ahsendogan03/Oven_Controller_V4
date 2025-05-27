/*
 * DWIN_Adress.h
 *
 *  Created on: Jan 23, 2025
 *      Author: Step
 */

#ifndef INC_DWIN_ADRESS_H_
#define INC_DWIN_ADRESS_H_

#include "main.h"


/*---------------------------- SAYFALAR ------------------------------*/

#define SURE_SONU_ADR				0x0053
#define DW_ARIZA_PAGE_ADR				85
#define DW_PISIRME_PAGE_ADR				2

/*---------------------------- MANUEL SAYFASI ------------------------------*/

#define DW_MANUEL_MOD_GIRIS_ADR			0x2000

#define DW_ILK_KULLANILAN_ADR			DW_UST_SICAKLIK_SET_ADR
#define DW_UST_SICAKLIK_SET_ADR			0x1000
#define DW_ALT_SICAKLIK_SET_ADR			0x1002
#define DW_UST_ON_SET_ADR				0x1004
#define DW_UST_ARKA_SET_ADR				0x1006
#define DW_ALT_SET_ADR					0x1008
#define DW_PISIRME_SURESI_ADR			0x100A
#define DW_PISIRME_SURESI_SN_ADR		0x100E
#define DW_BUHAR_SURESI_ADR				0x100C
#define DW_MCP9700_ADR					0x100F
#define DW_UST_SICAKLIK_ADR				0x1010
#define DW_ALT_SICAKLIK_ADR				0x1011
#define DW_PISIRME_BASLATMA_ADR			0x1012
#define DW_PISIRME_SONLANDIRMA_ADR		0x1014
#define DW_BUHAR_HAZIRLAMA_ADR			0x1015
#define DW_BUHAR_PUSKURTME_ADR			0x1018
#define DW_LAMBA_ADR					0x1019
#define DW_TURBO_ADR					0x101A
#define DW_PISIRME_ALARM_SUSTURMA_ADR	0x101B

/*------------------------------- ANIMSAYON ADDRESS  --------------------------------*/

#define DW_UST_SICAKLIK_ANIM			0x101C
#define DW_ALT_SICAKLIK_ANIM			0x101E
#define DW_UST_ON_ANIM					0x1020
#define DW_UST_ARKA_ANIM				0x1022
#define DW_ALT_ANIM						0x1024
#define DW_BUHAR_HAZIR_ANIM				0x1016

/*---------------------------- ORTAK AYARLAMA SAYFASI	 ------------------------------*/

#define DW_UST_SICAKLIK_SET_ORT_ADR		0x155B
#define DW_ALT_SICAKLIK_SET_ORT_ADR		0x155D
#define DW_UST_ON_SET_ORT_ADR			0x155F
#define DW_UST_ARKA_SET_ORT_ADR			0x1561
#define DW_ALT_SET_ORT_ADR				0x1563
#define DW_PISIRME_SURESI_ORT_ADR		0x1565
#define DW_BUHAR_SURESI_ORT_ADR			0x1567
#define DW_UST_SICAKLIK_SET_ONAY_ADR	0x1569
#define DW_ALT_SICAKLIK_SET_ONAY_ADR	0x156A
#define DW_UST_ON_SET_ONAY_ADR			0x156B
#define DW_UST_ARKA_SET_ONAY_ADR		0x156C
#define DW_ALT_SET_ONAY_ADR				0x156D
#define DW_PISIRME_SURESI_SET_ONAY_ADR	0x156E
#define DW_BUHAR_SURESI_SET_ONAY_ADR	0x156F

/*---------------------------- PARAMETRELER SAYFASI	 ------------------------------*/

#define DW_PARAMETRE_PAGE_ADR			0x17A8
#define DW_PARAMETRE_PSW				7251
#define DW_LAMBA_SURESI_ADR				0x17A9
#define DW_UST_ON_ISITICI_BANDI_ADR		0x17BC
#define DW_UST_ARKA_ISITICI_BANDI_ADR	0x17BE
#define DW_ALT_ISITICI_BANDI_ADR		0x17C0
#define	DW_ISITICI_UST_HIS_ADR			0x17C2
#define DW_ISITICI_ALT_HIS_ADR			0x17C4
#define DW_ISITICI_PERIOD_ADR			0x17C6

/*---------------------------- ARIZA ALARM SAYFASI	 ------------------------------*/

#define DW_TC1_ARIZA_ADR				0x178E
#define DW_TC2_ARIZA_ADR				0x178F
#define DW_TC3_ARIZA_ADR				0x1790
#define DW_MCP9700_ARIZA_ADR			0x1791
#define DW_EEPROM_ARIZA_ADR				0x1792
#define DW_ASIRI_SICAKLIK_ARIZA_ADR		0x1793
#define DW_ARIZA_ALARM_SUSTURMA_ADR		0x1794

/*--------------------------------- RECETELER -----------------------------------*/

#define DW_RECETE_AMOUNT				100
#define DW_RECETE_SIZE					20
#define DW_RECETE_RESIM_ILK_ADR			0x10FB
#define DW_RECETE_ISIM_ILK_ADR			0x115F

#define DW_RECETE_ILK_ADR				0x1097
#define DW_RECETE_SON_ADR				0x10FA




#endif /* INC_DWIN_ADRESS_H_ */
