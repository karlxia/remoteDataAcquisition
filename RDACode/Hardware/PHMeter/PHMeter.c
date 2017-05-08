#include "PHMeter.h"
#include "oled.h"
uint8_t PHMeterReg=0x00;
uint8_t PHMeterCMDBuf[PHMETER_CMDBUF_SIZE];
uint8_t PHMeterDataBuf[PHMETER_DATABUF_SIZE];
uint8_t PHMeteErrFC=0x00;
uint8_t PHMeteErrCODE=0x00;
uint16_t	PHData=0x0000;
uint16_t	TData=0x0000;
uint16_t	ORPData=0x0000;
uint16_t	PHThreshold=0x0000;
uint16_t	TThreshold=0x0000;
uint16_t	ORPThreshold=0x0000;
uint16_t	PHMeterCode=0x0000;
void PHMeterErrorHandle(void){
		OLED_ShowString(PHMETER_DISP_TITAL_X,PHMETER_DISP_TITAL_Y,"PHERR",16); 
	
}

void PHMeterTOHandle(void){
		OLED_ShowString(PHMETER_DISP_TITAL_X,PHMETER_DISP_TITAL_Y,"PHTO",16); 
	
}

void PHMeterCRCHandle(void){
		OLED_ShowString(PHMETER_DISP_TITAL_X,PHMETER_DISP_TITAL_Y,"PHCRC",16); 
	
}


void PHMeterDisplay(void){
		OLED_ShowString(PHMETER_DISP_TITAL_X,PHMETER_DISP_TITAL_Y,"PHMeter",16);  
	
		OLED_ShowString(PHMETER_DISP_ITEM_X,PHMETER_DISP_ITEM_Y,"ITEM",15);  
		OLED_ShowString(PHMETER_DISP_VALUE_X,PHMETER_DISP_VALUE_Y,"VAL",15);  
		OLED_ShowString(PHMETER_DISP_THRESHOLD_X,PHMETER_DISP_THRESHOLD_Y,"THR",15);  
	
		OLED_ShowString(PHMETER_DISP_PH_X,PHMETER_DISP_PH_Y," PH",15);  
		OLED_ShowString(PHMETER_DISP_T_X,PHMETER_DISP_T_Y,"  T",15);  
		OLED_ShowString(PHMETER_DISP_ORP_X,PHMETER_DISP_ORP_Y,"ORP",15);  
	
		OLED_ShowNum(PHMETER_DISP_CODE_X,PHMETER_DISP_CODE_Y,PHMeterCode,2,16);  
		OLED_ShowNum(PHMETER_DISP_PHVALUE_X,PHMETER_DISP_PHVALUE_Y,PHData/100,2,15);
		OLED_ShowString(PHMETER_DISP_PHVALUE_X+2*6+1,PHMETER_DISP_PHVALUE_Y,".",15);
		OLED_ShowNum(PHMETER_DISP_PHVALUE_X+2*6+6,PHMETER_DISP_PHVALUE_Y,PHData%100/10,1,15);
		OLED_ShowNum(PHMETER_DISP_PHVALUE_X+3*6+6,PHMETER_DISP_PHVALUE_Y,PHData%10,1,15);
		OLED_ShowNum(PHMETER_DISP_TVALUE_X,PHMETER_DISP_TVALUE_Y,TData/10,2,15);
		OLED_ShowString(PHMETER_DISP_PHVALUE_X+2*6+1,PHMETER_DISP_TVALUE_Y,".",15);
		OLED_ShowNum(PHMETER_DISP_TVALUE_X+2*6+6,PHMETER_DISP_TVALUE_Y,TData%10,1,15);
		OLED_ShowNum(PHMETER_DISP_ORPVALUE_X,PHMETER_DISP_ORPVALUE_Y,ORPData,4,15);
	
		OLED_ShowNum(PHMETER_DISP_PHTHRESHOLD_X,PHMETER_DISP_PHTHRESHOLD_Y,PHThreshold,4,15);
		OLED_ShowNum(PHMETER_DISP_TTHRESHOLD_X,PHMETER_DISP_TTHRESHOLD_Y,TThreshold,4,15);
		OLED_ShowNum(PHMETER_DISP_ORPTHRESHOLD_X,PHMETER_DISP_ORPTHRESHOLD_Y,ORPThreshold,4,15);
	
}

void PHMeterRequestData(void){
	uint16_t CRC16DATA=0X0000;
	PHMeterCMDBuf[PHMETER_ADDR_OFF]		=PHMeterAddr;
	PHMeterCMDBuf[PHMETER_FC_OFF]			=PHMETER_FC;
	CRC16DATA=CRC16(PHMeterCMDBuf,PHMETER_CRCLEN_OFF);
	PHMeterCMDBuf[PHMETER_CRCLEN_OFF]	=CRC16DATA>>8;
	PHMeterCMDBuf[PHMETER_CRCLEN_OFF+1]	=CRC16DATA&0xff;
	RS485_Send_Data(PHMeterCMDBuf,PHMETER_CMDBUF_SIZE);
}
	 
void PHMeterRequestPH(void){
/*	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}*/
	PHMeterCMDBuf[PHMETER_OFFSET_OFF+1]	=PHMETER_PH_OFFSET&0xff;
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_PH_OFFSET>>8;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF+1]=PHMETER_PH_DATACNT&0xff;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]=PHMETER_PH_DATACNT>>8;	
	PHMeterReg|=PHMETER_PH_PEND;
	PHMeterRequestData();
}

void PHMeterRequestT(void){
/*	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}*/
	PHMeterCMDBuf[PHMETER_OFFSET_OFF+1]	=PHMETER_T_OFFSET&0xff;
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_T_OFFSET>>8;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF+1]	=PHMETER_T_DATACNT&0xff;	
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_T_DATACNT>>8;	
	PHMeterReg|=PHMETER_T_PEND;
	PHMeterRequestData();
}
 
void PHMeterRequestPHT(void){
/*	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}*/
	PHMeterCMDBuf[PHMETER_OFFSET_OFF+1]	=PHMETER_PHT_OFFSET&0xff;
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_PHT_OFFSET>>8;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF+1]	=PHMETER_PHT_DATACNT&0xff;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=(PHMETER_PHT_DATACNT)>>8;	
	PHMeterReg|=PHMETER_PHT_PEND;
	PHMeterRequestData();
}
 
void PHMeterRequestORP(void){
/*	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}*/
	PHMeterCMDBuf[PHMETER_OFFSET_OFF+1]	=PHMETER_ORP_OFFSET&0xff;
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_ORP_OFFSET>>8;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF+1]	=PHMETER_ORP_DATACNT&0xff;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_ORP_DATACNT>>8;	
	PHMeterReg|=PHMETER_ORP_PEND;
	PHMeterRequestData();
}

void PHMeterRequestORPT(void){
/*	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}*/
	PHMeterCMDBuf[PHMETER_OFFSET_OFF+1]	=PHMETER_TORP_OFFSET&0xff;
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_TORP_OFFSET>>8;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF+1]	=PHMETER_TORP_DATACNT&0xff;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=(PHMETER_TORP_DATACNT)>>8;	
	PHMeterReg|=PHMETER_ORPT_PEND;
	PHMeterRequestData();
}

void PHMeterErrReceiveHandle(void){
	uint8_t Temp=0x00;
	uint16_t rCRC=0x0000;			//readCRC
	uint16_t cCRC=0x0000;			//calculateCRC
	Temp=PHMeterReg&(~PHMETER_RBUF_UPDATE);
	rCRC=PHMeterDataBuf[PHMETER_ERRCRC_OFF]<<8|PHMeterDataBuf[PHMETER_ERRCRC_OFF+1];
	cCRC=CRC16(PHMeterDataBuf,PHMETER_ERRCRC_OFF);
	if(rCRC!=cCRC){
		PHMeterCRCHandle();//CRCERR
		return ;
	}	
	if(PHMeterDataBuf[PHMETER_ADDR_OFF]!=PHMeterAddr){
		PHMeterErrorHandle();
	}
	PHMeteErrFC=PHMeterDataBuf[PHMETER_FC_OFF]&~0x80;
	PHMeteErrCODE=PHMeterDataBuf[PHMETER_ERRCRC_OFF];
	OLED_Clear();
	OLED_ShowString(PHMETER_DISP_TITAL_X,PHMETER_DISP_TITAL_Y,"PHMeter",16);  //L1
	OLED_ShowString(PHMETER_DISP_CODE_X,PHMETER_DISP_CODE_Y,"Error",16);  	//error	
	OLED_ShowString(PHMETER_DISP_PH_X,PHMETER_DISP_PH_Y,"  ErrFC",15);  
	OLED_ShowString(PHMETER_DISP_T_X,PHMETER_DISP_T_Y,"ErrCODE",15);  
	OLED_ShowNum(PHMETER_DISP_PHVALUE_X,PHMETER_DISP_PHVALUE_Y,PHMeteErrFC,4,15);
	OLED_ShowNum(PHMETER_DISP_TVALUE_X,PHMETER_DISP_TVALUE_Y,PHMeteErrCODE,4,15); 
	switch (Temp){
		case PHMETER_PH_PEND:
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_PH_PEND);
			PHMeterRequestPH();
			break;
		case PHMETER_T_PEND:
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_T_PEND);
			PHMeterRequestT();
			break;
		case PHMETER_PHT_PEND:
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_PH_PEND);
			PHMeterReg&=(~PHMETER_T_PEND);
			PHMeterRequestPHT();
			break;
		case PHMETER_ORP_PEND:
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_ORP_PEND);
			PHMeterRequestORP();
			break;
		case PHMETER_ORPT_PEND:
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_ORP_PEND);
			PHMeterReg&=(~PHMETER_T_PEND);
			PHMeterRequestORPT();
			break;
		default:	;
	}
	PHMeterReg=0x00;
}

void PHMeterDataReceiveHandle(void){
	uint8_t Temp=0x00;
	uint16_t CRCData=0x0000;
	Temp=PHMeterReg&(~PHMETER_RBUF_UPDATE);
	if(PHMeterDataBuf[PHMETER_ADDR_OFF]!=PHMeterAddr){
		PHMeterErrorHandle();
	}
	if(PHMeterDataBuf[PHMETER_FC_OFF]!=PHMETER_FC){
		PHMeterErrorHandle();
	}
	switch (Temp){
		case PHMETER_PH_PEND:
			CRCData=PHMeterDataBuf[PHMETER_SCRCLEN_OFF]<<8|PHMeterDataBuf[PHMETER_SCRCLEN_OFF+1];
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_PH_DATALEN){
				PHMeterErrorHandle();
			}
			if(CRCData!=CRC16(PHMeterDataBuf,PHMETER_SCRCLEN_OFF)){
				PHMeterCRCHandle();
			}
			PHData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_PH_PEND);
			break;
		case PHMETER_T_PEND:
			CRCData=PHMeterDataBuf[PHMETER_SCRCLEN_OFF]<<8|PHMeterDataBuf[PHMETER_SCRCLEN_OFF+1];
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_T_DATALEN){
				PHMeterErrorHandle();
			}
			if(CRCData!=CRC16(PHMeterDataBuf,PHMETER_SCRCLEN_OFF)){
				PHMeterCRCHandle();
			}
			TData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_T_PEND);
			break;
		case PHMETER_PHT_PEND:
			CRCData=PHMeterDataBuf[PHMETER_DCRCLEN_OFF]<<8|PHMeterDataBuf[PHMETER_DCRCLEN_OFF+1];
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_PHT_DATALEN){
				PHMeterErrorHandle();
			}
			if(CRCData!=CRC16(PHMeterDataBuf,PHMETER_DCRCLEN_OFF)){
				PHMeterCRCHandle();
			}
			PHData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			TData=PHMeterDataBuf[PHMETER_DATA_OFF+2]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+3];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_PH_PEND);
			PHMeterReg&=(~PHMETER_T_PEND);
			break;
		case PHMETER_ORP_PEND:
			CRCData=PHMeterDataBuf[PHMETER_SCRCLEN_OFF]<<8|PHMeterDataBuf[PHMETER_SCRCLEN_OFF+1];
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_ORP_DATALEN){
				PHMeterErrorHandle();
			}
			if(CRCData!=CRC16(PHMeterDataBuf,PHMETER_SCRCLEN_OFF)){
				PHMeterCRCHandle();
			}
			ORPData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_ORP_PEND);
			break;
		case PHMETER_ORPT_PEND:
			CRCData=PHMeterDataBuf[PHMETER_DCRCLEN_OFF]<<8|PHMeterDataBuf[PHMETER_DCRCLEN_OFF+1];
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_TORP_DATALEN){
				PHMeterErrorHandle();
			}
			if(CRCData!=CRC16(PHMeterDataBuf,PHMETER_DCRCLEN_OFF)){
				PHMeterCRCHandle();
			}
			TData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			ORPData=PHMeterDataBuf[PHMETER_DATA_OFF+2]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+3];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_ORP_PEND);
			PHMeterReg&=(~PHMETER_T_PEND);
			break;
		default:	;
	}
	PHMeterReg=0x00;
}

void PHMeterReceiveHandle(void){
	if(PHMeterDataBuf[PHMETER_FC_OFF]&0x80){
		PHMeterErrReceiveHandle();
	}else{
		PHMeterDataReceiveHandle();
	}
}
void PHMeterCheck(void){
	if(PHMeterReg&PHMETER_RBUF_UPDATE){
		PHMeterReceiveHandle();
		PHMeterReg&=~PHMETER_RBUF_UPDATE;
	}
	
}












