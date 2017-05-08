#include "DissolvedOxygenMeter.h"
#include "oled.h"
uint8_t DOMeterReg=0x00;
uint8_t DOMeterCMDBuf[DOMETER_CMDBUF_SIZE];
uint8_t DOMeterDataBuf[DOMETER_DATABUF_SIZE];
uint8_t DOMeterErrFC=0x00;
uint8_t DOMeterErrCODE=0x00;
uint16_t	DOData=0x0000;
uint16_t	DOTData=0x0000;
uint16_t	HALMData=0x0000;
uint16_t	LALMData=0x0000;
uint16_t	LTCData=0x0000;
uint16_t	STAData=0x0000;
uint16_t 	setHALM=0xffff;
uint16_t 	setLALM=0x0000;
uint16_t 	setLTC=0x00ff;
void DOMeterErrorHandle(void){
	OLED_ShowString(DOMETER_DISP_TITAL_X,DOMETER_DISP_TITAL_Y,"DOERR",16);  //L1
}

void DOMeterCRCHandle(void){
	OLED_ShowString(DOMETER_DISP_TITAL_X,DOMETER_DISP_TITAL_Y,"DOCRC",16);  //L1
}

void DOMeterTOHandle(void){
	OLED_ShowString(DOMETER_DISP_TITAL_X,DOMETER_DISP_TITAL_Y,"DOTO",16);  //L1
}


void DOMeterDisplay(void){
		OLED_ShowString(DOMETER_DISP_TITAL_X,DOMETER_DISP_TITAL_Y,"DOMeter",16);  //L1
		OLED_ShowNum(DOMETER_DISP_CODE_X,DOMETER_DISP_CODE_Y,STAData,2,16);  	//STA	
	
		OLED_ShowString(DOMETER_DISP_ITEM_X,DOMETER_DISP_ITEM_Y,"ITEM",15);  	//L2
		OLED_ShowString(DOMETER_DISP_MIN_X,DOMETER_DISP_MIN_Y,"MIN",15); 
		OLED_ShowString(DOMETER_DISP_VALUE_X,DOMETER_DISP_VALUE_Y,"VAL",15);  
		OLED_ShowString(DOMETER_DISP_MAX_X,DOMETER_DISP_MAX_Y,"MAX",15);  
	
		OLED_ShowString(DOMETER_DISP_DO_X,DOMETER_DISP_DO_Y," DO",15);  	//ITEM	
		OLED_ShowString(DOMETER_DISP_T_X,DOMETER_DISP_T_Y,"  T",15);  
		OLED_ShowString(DOMETER_DISP_LTC_X,DOMETER_DISP_LTC_Y,"LTC",15);  
	
		OLED_ShowNum(DOMETER_DISP_DOMIN_X,DOMETER_DISP_DOMIN_Y,LALMData/10,2,15);	//MIN
		OLED_ShowString(DOMETER_DISP_DOMIN_X+2*6+1,DOMETER_DISP_DOMIN_Y,".",15);
		OLED_ShowNum(DOMETER_DISP_DOMIN_X+3*6,DOMETER_DISP_DOMIN_Y,LALMData%10,1,15);
	
		OLED_ShowNum(DOMETER_DISP_DOVALUE_X,DOMETER_DISP_DOVALUE_Y,DOData/1000,1,15);	//VALUE
		OLED_ShowString(DOMETER_DISP_DOVALUE_X+6+1,DOMETER_DISP_DOVALUE_Y,".",15);
		OLED_ShowNum(DOMETER_DISP_DOVALUE_X+2*6,DOMETER_DISP_DOVALUE_Y,DOData%1000/100,1,15);
		OLED_ShowNum(DOMETER_DISP_DOVALUE_X+3*6,DOMETER_DISP_DOVALUE_Y,DOData%100/10,1,15);
		OLED_ShowNum(DOMETER_DISP_DOVALUE_X+4*6,DOMETER_DISP_DOVALUE_Y,DOData%10,1,15);
		OLED_ShowNum(DOMETER_DISP_TVALUE_X,DOMETER_DISP_TVALUE_Y,DOTData/100,2,15);
		OLED_ShowString(DOMETER_DISP_TVALUE_X+2*6+1,DOMETER_DISP_TVALUE_Y,".",15);
		OLED_ShowNum(DOMETER_DISP_TVALUE_X+3*6,DOMETER_DISP_TVALUE_Y,DOTData%100/10,1,15);
		OLED_ShowNum(DOMETER_DISP_TVALUE_X+4*6,DOMETER_DISP_TVALUE_Y,DOTData/10,1,15);
		OLED_ShowNum(DOMETER_DISP_LTCVALUE_X,DOMETER_DISP_LTCVALUE_Y,LTCData/10,2,15);
		OLED_ShowString(DOMETER_DISP_LTCVALUE_X+2*6+1,DOMETER_DISP_LTCVALUE_Y,".",15);
		OLED_ShowNum(DOMETER_DISP_LTCVALUE_X+3*6,DOMETER_DISP_LTCVALUE_Y,LTCData%10,1,15);

		OLED_ShowNum(DOMETER_DISP_DOMAX_X,DOMETER_DISP_DOMAX_Y,HALMData/10,2,15);	//MAX
		OLED_ShowString(DOMETER_DISP_DOMAX_X+2*6+1,DOMETER_DISP_DOMAX_Y,".",15);	
		OLED_ShowNum(DOMETER_DISP_DOMAX_X+3*6,DOMETER_DISP_DOMAX_Y,HALMData%10,1,15);	
}

void DOMeterRequestData(void){
	uint16_t CRCDATA=0X0000;
	DOMeterCMDBuf[DOMETER_ADDR_OFF]		=DOMeterAddr;
	DOMeterCMDBuf[DOMETER_FC_OFF]		=DOMETER_REQ_FC;
	DOMeterCMDBuf[DOMETER_START_OFF]	=0X00;
	DOMeterCMDBuf[DOMETER_START_OFF+1]	=0X00;
	DOMeterCMDBuf[DOMETER_REGCNT_OFF]	=0X00;
	DOMeterCMDBuf[DOMETER_REGCNT_OFF+1]	=0X06;
	
	CRCDATA=CRC16(DOMeterCMDBuf,DOMETER_REQCRC_OFF);
	DOMeterCMDBuf[DOMETER_REQCRC_OFF+1]	=CRCDATA&0XFF;
	DOMeterCMDBuf[DOMETER_REQCRC_OFF]	=CRCDATA>>8;
	RS485_Send_Data(DOMeterCMDBuf,DOMETER_REQCRC_OFF+2);
	DOMeterReg|=DOMETER_REQ_PEND;
}
	 
void DOMeterWriteReg(uint16_t HALM,uint16_t LALM,uint16_t LTC){
	uint16_t CRCDATA=0X0000;
	DOMeterCMDBuf[DOMETER_ADDR_OFF]			=DOMeterAddr;
	DOMeterCMDBuf[DOMETER_FC_OFF]				=DOMETER_WREG_FC;
	DOMeterCMDBuf[DOMETER_START_OFF]		=0X00;
	DOMeterCMDBuf[DOMETER_START_OFF+1]	=0X00;
	DOMeterCMDBuf[DOMETER_REGCNT_OFF]		=0X00;
	DOMeterCMDBuf[DOMETER_REGCNT_OFF+1]	=0X03;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF]	=0X06;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+1]=HALM>>8;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+2]=HALM&0XFF;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+3]=LALM>>8;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+4]=LALM&0XFF;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+5]=LTC>>8;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+6]=LTC&0XFF;
	
	CRCDATA=CRC16(DOMeterCMDBuf,DOMETER_DATANUM_OFF+7);
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+7]	=CRCDATA>>8;
	DOMeterCMDBuf[DOMETER_DATANUM_OFF+8]	=CRCDATA&0XFF;
	RS485_Send_Data(DOMeterCMDBuf,DOMETER_CMDBUF_SIZE);
	DOMeterReg|=DOMETER_WRITE_PEND;
}

void DOMeterErrReceiveHandle(void){
	uint8_t Temp=0x00;
	uint16_t rCRC=0x0000;			//readCRC
	uint16_t cCRC=0x0000;			//calculateCRC
	Temp=DOMeterReg&(~DOMETER_RBUF_UPDATE);
	rCRC=DOMeterDataBuf[DOMETER_ERRCRC_OFF]<<8|DOMeterDataBuf[DOMETER_ERRCRC_OFF+1];
	cCRC=CRC16(DOMeterDataBuf,DOMETER_ERRCRC_OFF);
	if(rCRC!=cCRC){
		DOMeterCRCHandle();//CRCERR
		return ;
	}	
	if(DOMeterDataBuf[DOMETER_ADDR_OFF]!=DOMeterAddr){
		DOMeterErrorHandle();	//Meter addr err
	}
	DOMeterErrFC=DOMeterDataBuf[DOMETER_FC_OFF]&~0x80;
	DOMeterErrCODE=DOMeterDataBuf[DOMETER_CODE_OFF];
	OLED_Clear();
	OLED_ShowString(DOMETER_DISP_TITAL_X,DOMETER_DISP_TITAL_Y,"DOMeter",16);  //L1
	OLED_ShowString(DOMETER_DISP_CODE_X,DOMETER_DISP_CODE_Y,"Error",16);  	//error	
	OLED_ShowString(DOMETER_DISP_DO_X,DOMETER_DISP_DO_Y,"  ErrFC",15);  	//ITEM	
	OLED_ShowString(DOMETER_DISP_T_X,DOMETER_DISP_T_Y,"ErrCODE",15);  
	OLED_ShowNum(DOMETER_DISP_DOVALUE_X,DOMETER_DISP_DOVALUE_Y,DOMeterErrFC,4,15);	//VALUE
	OLED_ShowNum(DOMETER_DISP_TVALUE_X,DOMETER_DISP_TVALUE_Y,DOMeterErrCODE,4,15);
	switch (Temp){
		case DOMETER_REQ_PEND:		
			DOMeterReg&=(~DOMETER_RBUF_UPDATE);
			DOMeterReg&=(~DOMETER_REQ_PEND);
			DOMeterRequestData();
			break;
		case DOMETER_WRITE_PEND:
			DOMeterReg&=(~DOMETER_RBUF_UPDATE);
			DOMeterReg&=(~DOMETER_WRITE_PEND);
			DOMeterWriteReg(setHALM,setLALM,setLTC);
			break;
		
		default:	;
	}
}

void DOMeterDataReceiveHandle(void){
	uint16_t rCRC=0x0000;			//readCRC
	uint16_t cCRC=0x0000;			//calculateCRC
	if(DOMeterDataBuf[DOMETER_ADDR_OFF]!=DOMeterAddr){
		DOMeterErrorHandle();
	}
	switch (DOMeterDataBuf[DOMETER_FC_OFF]){
		case DOMETER_REQ_FC:		
			rCRC=DOMeterDataBuf[DOMETER_DATACRC_OFF]<<8|DOMeterDataBuf[DOMETER_DATACRC_OFF+1];
			cCRC=CRC16(DOMeterDataBuf,DOMETER_DATACRC_OFF);
			if(rCRC!=cCRC){
				DOMeterCRCHandle();//CRCERR
				return ;
			}	
			DOData	=DOMeterDataBuf[DOMETER_DOH_OFFSET]<<8|DOMeterDataBuf[DOMETER_DOL_OFFSET];
			DOTData		=DOMeterDataBuf[DOMETER_TH_OFFSET]<<8|DOMeterDataBuf[DOMETER_TL_OFFSET];
			HALMData=DOMeterDataBuf[DOMETER_HALMH_OFFSET]<<8|DOMeterDataBuf[DOMETER_HALML_OFFSET];
			LALMData=DOMeterDataBuf[DOMETER_LALMH_OFFSET]<<8|DOMeterDataBuf[DOMETER_LALML_OFFSET];
			LTCData	=DOMeterDataBuf[DOMETER_LTCH_OFFSET]<<8|DOMeterDataBuf[DOMETER_LTCL_OFFSET];
			STAData	=DOMeterDataBuf[DOMETER_ALMSTA_OFFSET];
			DOMeterReg&=(~DOMETER_RBUF_UPDATE);
			DOMeterReg&=(~DOMETER_REQ_PEND);
			break;
		case DOMETER_WREG_FC:
			rCRC=DOMeterDataBuf[6]<<8|DOMeterDataBuf[6+1];
			cCRC=CRC16(DOMeterDataBuf,6);
			if(rCRC!=cCRC){
				DOMeterCRCHandle();//CRCERR
				return ;
			}	
			DOMeterReg&=(~DOMETER_RBUF_UPDATE);
			DOMeterReg&=(~DOMETER_WRITE_PEND);
			break;
		
		default:	;
	}
}

void DOMeterReceiveHandle(void){
	if(DOMeterDataBuf[DOMETER_FC_OFF]&0x80){
		DOMeterErrReceiveHandle();
	}else{
		DOMeterDataReceiveHandle();
	}
}

void DOMeterCheck(void){
	if(DOMeterReg&DOMETER_RBUF_UPDATE){
		DOMeterReceiveHandle();
		DOMeterReg&=~DOMETER_RBUF_UPDATE;
	}
	
	
}

