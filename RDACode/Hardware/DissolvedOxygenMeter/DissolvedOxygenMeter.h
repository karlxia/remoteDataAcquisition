#ifndef __DISSOLVEDOXYGENMETER_H
#define __DISSOLVEDOXYGENMETER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_karlboard.h"
#include "oled.h"
#include "CRC16.h"
#include "rs485.h"
#include "main.h"
#define DOMETER_REQ_FC			0x03
#define DOMETER_WREG_FC			0x10
#define DOMeterAddr					0x03
#define DOMeterTO						0x05
//CMD BUF OFF
//REQUIRE DATA CMD
#define DOMETER_ADDR_OFF		0
#define DOMETER_FC_OFF			DOMETER_ADDR_OFF+1		//1
#define DOMETER_START_OFF		DOMETER_FC_OFF+1		//2
#define DOMETER_REGCNT_OFF		DOMETER_START_OFF+2		//4
#define DOMETER_REQCRC_OFF		DOMETER_REGCNT_OFF+2	//6
//WRITE REGISTER CDM
#define DOMETER_DATANUM_OFF		DOMETER_REGCNT_OFF+2	//6

//DATA BUF OFF
//DATA BACK
#define DOMETER_DATACNT_OFF		DOMETER_ADDR_OFF+2		//2
#define DOMETER_DATA_OFF		DOMETER_DATACNT_OFF+1	//3
#define DOMETER_DATACRC_OFF		DOMETER_DATA_OFF+12		//15
//ERROR BACK
#define DOMETER_CODE_OFF		DOMETER_ADDR_OFF+2		//2
#define DOMETER_ERRCRC_OFF		DOMETER_CODE_OFF+1		//3


//BUF SIZE
#define DOMETER_CMDBUF_SIZE		15	//15
#define DOMETER_DATABUF_SIZE	17	//17

//DATA OFFSET
#define DOMETER_DOH_OFFSET		DOMETER_DATA_OFF//1+2
#define DOMETER_DOL_OFFSET		DOMETER_DOH_OFFSET+1	//2
#define DOMETER_TH_OFFSET		DOMETER_DOH_OFFSET+2	//3
#define DOMETER_TL_OFFSET		DOMETER_TH_OFFSET+1		//4
#define DOMETER_HALMH_OFFSET	DOMETER_TH_OFFSET+2		//5
#define DOMETER_HALML_OFFSET	DOMETER_HALMH_OFFSET+1	//6
#define DOMETER_LALMH_OFFSET	DOMETER_HALMH_OFFSET+2	//7
#define DOMETER_LALML_OFFSET	DOMETER_LALMH_OFFSET+1	//8
#define DOMETER_LTCH_OFFSET		DOMETER_LALMH_OFFSET+2	//9
#define DOMETER_LTCL_OFFSET		DOMETER_LTCH_OFFSET+1	//10
#define DOMETER_ALMSTA_OFFSET	DOMETER_LTCH_OFFSET+2	//11


//DOMETER REGISTER
#define DOMETER_REQ_PEND					0x01
#define DOMETER_WRITE_PEND					0x02

#define DOMETER_RBUF_UPDATE			0x80

//display
#define	DOMETER_X_LEN		32
#define	DOMETER_Y_LEN		1
//L1
#define DOMETER_DISP_TITAL_X				0
#define DOMETER_DISP_TITAL_Y				0

#define DOMETER_DISP_CODE_X					DOMETER_DISP_TITAL_X+DOMETER_X_LEN*2
#define DOMETER_DISP_CODE_Y					DOMETER_DISP_TITAL_Y
//L2
#define DOMETER_DISP_ITEM_X					DOMETER_DISP_TITAL_X	//ITEM
#define DOMETER_DISP_ITEM_Y					DOMETER_DISP_TITAL_Y+2

#define DOMETER_DISP_MIN_X					DOMETER_DISP_ITEM_X+DOMETER_X_LEN	//MIN
#define DOMETER_DISP_MIN_Y					DOMETER_DISP_ITEM_Y

#define DOMETER_DISP_VALUE_X				DOMETER_DISP_MIN_X+DOMETER_X_LEN	//VALUE
#define DOMETER_DISP_VALUE_Y				DOMETER_DISP_ITEM_Y

#define DOMETER_DISP_MAX_X					DOMETER_DISP_VALUE_X+DOMETER_X_LEN	//MAX
#define DOMETER_DISP_MAX_Y					DOMETER_DISP_ITEM_Y

//L3	DO
#define DOMETER_DISP_DO_X					DOMETER_DISP_ITEM_X
#define DOMETER_DISP_DO_Y					DOMETER_DISP_ITEM_Y+DOMETER_Y_LEN

#define DOMETER_DISP_DOMIN_X				DOMETER_DISP_MIN_X
#define DOMETER_DISP_DOMIN_Y				DOMETER_DISP_DO_Y

#define DOMETER_DISP_DOVALUE_X				DOMETER_DISP_VALUE_X
#define DOMETER_DISP_DOVALUE_Y				DOMETER_DISP_DO_Y

#define DOMETER_DISP_DOMAX_X				DOMETER_DISP_MAX_X
#define DOMETER_DISP_DOMAX_Y				DOMETER_DISP_DO_Y
//L4	T
#define DOMETER_DISP_T_X					DOMETER_DISP_ITEM_X
#define DOMETER_DISP_T_Y					DOMETER_DISP_DO_Y+DOMETER_Y_LEN

#define DOMETER_DISP_TMIN_X					DOMETER_DISP_MIN_X
#define DOMETER_DISP_TMIN_Y					DOMETER_DISP_T_Y

#define DOMETER_DISP_TVALUE_X				DOMETER_DISP_VALUE_X
#define DOMETER_DISP_TVALUE_Y				DOMETER_DISP_T_Y

#define DOMETER_DISP_TMAX_X					DOMETER_DISP_MAX_X
#define DOMETER_DISP_TMAX_Y					DOMETER_DISP_T_Y
//L5 	LTC
#define DOMETER_DISP_LTC_X					DOMETER_DISP_ITEM_X
#define DOMETER_DISP_LTC_Y					DOMETER_DISP_T_Y+DOMETER_Y_LEN

#define DOMETER_DISP_LTCMIN_X				DOMETER_DISP_MIN_X
#define DOMETER_DISP_LTCMIN_Y				DOMETER_DISP_LTC_Y

#define DOMETER_DISP_LTCVALUE_X				DOMETER_DISP_VALUE_X
#define DOMETER_DISP_LTCVALUE_Y				DOMETER_DISP_LTC_Y

#define DOMETER_DISP_LTCMAX_X				DOMETER_DISP_MAX_X
#define DOMETER_DISP_LTCMAX_Y				DOMETER_DISP_LTC_Y

extern uint16_t	DOData;
extern uint16_t	DOTData;
extern uint16_t	HALMData;
extern uint16_t	LALMData;
extern uint16_t	LTCData;
extern uint16_t	STAData;

extern uint16_t	DOMeterCode;
extern uint8_t DOMeterDataBuf[];
extern uint8_t DOMeterReg;
void DOMeterCheck(void);
void DOMeterDisplay(void);
void DOMeterRequestData(void);
void DOMeterWriteReg(uint16_t HALM,uint16_t LALM,uint16_t LTC);

#endif /* __DISSOLVEDOXYGENMETER_H */
