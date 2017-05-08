#ifndef __PHMETER_H
#define __PHMETER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_karlboard.h"
#include "oled.h"
#include "CRC16.h"
#include "rs485.h"
#include "main.h"
#define PHMETER_FC						0x03
#define PHMeterAddr						0x00
#define PHMeterTO							0x05
//CMD BUF OFF
#define PHMETER_ADDR_OFF			0
#define PHMETER_FC_OFF				PHMETER_ADDR_OFF+1		//1
#define PHMETER_OFFSET_OFF		PHMETER_FC_OFF+1			//2
#define PHMETER_DATACNT_OFF		PHMETER_OFFSET_OFF+2	//4
#define PHMETER_CRCLEN_OFF		PHMETER_DATACNT_OFF+2	//6
//DATA BUF OFF
#define PHMETER_DATALEN_OFF		PHMETER_ADDR_OFF+2		//2
#define PHMETER_DATA_OFF			PHMETER_DATALEN_OFF+1	//3
#define PHMETER_SCRCLEN_OFF		PHMETER_DATA_OFF+2		//5
#define PHMETER_DCRCLEN_OFF		PHMETER_DATA_OFF+4		//7
//ERR BUF OFF
#define PHMETER_CODE_OFF			PHMETER_ADDR_OFF+2		//2
#define PHMETER_ERRCRC_OFF		PHMETER_DATALEN_OFF+1	//3
//BUF SIZE
#define PHMETER_CMDBUF_SIZE		PHMETER_CRCLEN_OFF+2	//8
#define PHMETER_DATABUF_SIZE	PHMETER_DCRCLEN_OFF+2	//9
//PH CMD
#define PHMETER_PH_OFFSET			0x0000
#define PHMETER_PH_DATACNT		0x0001
#define PHMETER_PH_DATALEN		0x02
//T CMD
#define PHMETER_T_OFFSET			0x0001
#define PHMETER_T_DATACNT			0x0001
#define PHMETER_T_DATALEN			0x02
//PHT CMD
#define PHMETER_PHT_OFFSET		PHMETER_PH_OFFSET
#define PHMETER_PHT_DATACNT		PHMETER_PH_DATACNT+PHMETER_T_DATACNT
#define PHMETER_PHT_DATALEN		PHMETER_PH_DATALEN+PHMETER_T_DATALEN
//ORP CMD
#define PHMETER_ORP_OFFSET			0x0002
#define PHMETER_ORP_DATACNT			0x0001
#define PHMETER_ORP_DATALEN			0x02
//TORP CMD
#define PHMETER_TORP_OFFSET			PHMETER_T_OFFSET
#define PHMETER_TORP_DATACNT		PHMETER_T_DATACNT+PHMETER_ORP_DATACNT
#define PHMETER_TORP_DATALEN		PHMETER_T_DATALEN+PHMETER_ORP_DATALEN
//PHMETER REGISTER
#define PHMETER_PH_PEND					0x01
#define PHMETER_T_PEND					0x02
#define PHMETER_PHT_PEND				0x04
#define PHMETER_ORP_PEND				0x08
#define PHMETER_ORPT_PEND				0x10

#define PHMETER_RBUF_UPDATE			0x80

//display
#define	PHMETER_X_LEN		43
#define	PHMETER_Y_LEN		1
//L1
#define PHMETER_DISP_TITAL_X				0
#define PHMETER_DISP_TITAL_Y				0

#define PHMETER_DISP_CODE_X					PHMETER_DISP_TITAL_X+PHMETER_X_LEN*2
#define PHMETER_DISP_CODE_Y					PHMETER_DISP_TITAL_Y
//L2
#define PHMETER_DISP_ITEM_X					PHMETER_DISP_TITAL_X
#define PHMETER_DISP_ITEM_Y					PHMETER_DISP_TITAL_Y+2

#define PHMETER_DISP_VALUE_X				PHMETER_DISP_ITEM_X+PHMETER_X_LEN
#define PHMETER_DISP_VALUE_Y				PHMETER_DISP_ITEM_Y

#define PHMETER_DISP_THRESHOLD_X		PHMETER_DISP_VALUE_X+PHMETER_X_LEN
#define PHMETER_DISP_THRESHOLD_Y		PHMETER_DISP_ITEM_Y
//L3	PH
#define PHMETER_DISP_PH_X						PHMETER_DISP_ITEM_X
#define PHMETER_DISP_PH_Y						PHMETER_DISP_ITEM_Y+PHMETER_Y_LEN

#define PHMETER_DISP_PHVALUE_X			PHMETER_DISP_VALUE_X
#define PHMETER_DISP_PHVALUE_Y			PHMETER_DISP_PH_Y

#define PHMETER_DISP_PHTHRESHOLD_X	PHMETER_DISP_THRESHOLD_X
#define PHMETER_DISP_PHTHRESHOLD_Y	PHMETER_DISP_PH_Y
//L4	T
#define PHMETER_DISP_T_X						PHMETER_DISP_ITEM_X
#define PHMETER_DISP_T_Y						PHMETER_DISP_PH_Y+PHMETER_Y_LEN

#define PHMETER_DISP_TVALUE_X				PHMETER_DISP_VALUE_X
#define PHMETER_DISP_TVALUE_Y				PHMETER_DISP_T_Y

#define PHMETER_DISP_TTHRESHOLD_X		PHMETER_DISP_THRESHOLD_X
#define PHMETER_DISP_TTHRESHOLD_Y		PHMETER_DISP_T_Y
//L5	ORP
#define PHMETER_DISP_ORP_X					PHMETER_DISP_ITEM_X
#define PHMETER_DISP_ORP_Y					PHMETER_DISP_T_Y+PHMETER_Y_LEN

#define PHMETER_DISP_ORPVALUE_X			PHMETER_DISP_VALUE_X
#define PHMETER_DISP_ORPVALUE_Y			PHMETER_DISP_ORP_Y

#define PHMETER_DISP_ORPTHRESHOLD_X	PHMETER_DISP_THRESHOLD_X
#define PHMETER_DISP_ORPTHRESHOLD_Y	PHMETER_DISP_ORP_Y



extern uint16_t	PHData;
extern uint16_t	TData;
extern uint16_t	ORPData;
extern uint16_t	PHThreshold;
extern uint16_t	TThreshold;
extern uint16_t	ORPThreshold;
extern uint16_t	PHMeterCode;
extern uint8_t PHMeterDataBuf[];
extern uint8_t PHMeterReg;
void PHMeterCheck(void);
void PHMeterDisplay(void);
void PHMeterRequestPH(void);
void PHMeterRequestT(void);
void PHMeterRequestPHT(void);
void PHMeterRequestORP(void);
void PHMeterRequestORPT(void);
#endif /* __PHMETER_H */
