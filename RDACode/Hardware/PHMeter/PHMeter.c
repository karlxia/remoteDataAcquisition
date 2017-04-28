#include "PHMeter.h"
#include "oled.h"
uint8_t PHMeterReg=0x00;
uint8_t PHMeterCMDBuf[PHMETER_CMDBUF_SIZE];
uint8_t PHMeterDataBuf[PHMETER_DATABUF_SIZE];
uint16_t	PHData=0x0000;
uint16_t	TData=0x0000;
uint16_t	ORPData=0x0000;
uint16_t	PHThreshold=0x0000;
uint16_t	TThreshold=0x0000;
uint16_t	ORPThreshold=0x0000;
uint16_t	PHMeterCode=0x0000;
void PHMeterErrorHandle(void){
	
}

void PHMeterTOHandle(void){
	
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
		OLED_ShowNum(PHMETER_DISP_PHVALUE_X,PHMETER_DISP_PHVALUE_Y,PHMeterCode,4,15);
		OLED_ShowNum(PHMETER_DISP_TVALUE_X,PHMETER_DISP_TVALUE_Y,PHMeterCode,4,15);
		OLED_ShowNum(PHMETER_DISP_ORPVALUE_X,PHMETER_DISP_ORPVALUE_Y,PHMeterCode,4,15);
	
		OLED_ShowNum(PHMETER_DISP_PHTHRESHOLD_X,PHMETER_DISP_PHTHRESHOLD_Y,PHThreshold,4,15);
		OLED_ShowNum(PHMETER_DISP_TTHRESHOLD_X,PHMETER_DISP_TTHRESHOLD_Y,TThreshold,4,15);
		OLED_ShowNum(PHMETER_DISP_ORPTHRESHOLD_X,PHMETER_DISP_ORPTHRESHOLD_Y,ORPThreshold,4,15);
	
}

void PHMeterRequestData(void){
	PHMeterCMDBuf[PHMETER_ADDR_OFF]		=PHMeterAddr;
	PHMeterCMDBuf[PHMETER_FC_OFF]			=PHMETER_FC;
	PHMeterCMDBuf[PHMETER_CRCLEN_OFF]	=CRC16(PHMeterCMDBuf,PHMETER_CRCLEN_OFF);
	RS485_Send_Data(PHMeterCMDBuf,PHMETER_CMDBUF_SIZE);
}
	 
void PHMeterRequestPH(void){
	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_PH_OFFSET;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_PH_DATACNT;	
	PHMeterReg|=PHMETER_PH_PEND;
	PHMeterRequestData();
}

void PHMeterRequestT(void){
	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_T_OFFSET;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_T_DATACNT;	
	PHMeterReg|=PHMETER_T_PEND;
	PHMeterRequestData();
}
 
void PHMeterRequestPHT(void){
	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_PHT_OFFSET;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_PHT_DATACNT;	
	PHMeterReg|=PHMETER_PHT_PEND;
	PHMeterRequestData();
}
 
void PHMeterRequestORP(void){
	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_ORP_OFFSET;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_ORP_DATACNT;	
	PHMeterReg|=PHMETER_ORP_PEND;
	PHMeterRequestData();
}

void PHMeterRequestORPT(void){
	uint8_t Temp=0x00;
	while(PHMeterReg){
		Temp++;
		if(Temp>>PHMeterTO){
			PHMeterTOHandle();
			return ;
		}
	}
	PHMeterCMDBuf[PHMETER_OFFSET_OFF]	=PHMETER_TORP_OFFSET;
	PHMeterCMDBuf[PHMETER_DATACNT_OFF]	=PHMETER_TORP_DATACNT;	
	PHMeterReg|=PHMETER_ORPT_PEND;
	PHMeterRequestData();
}

void PHMeterReceiveHandle(void){
	uint8_t Temp=0x00;
	Temp=PHMeterReg&(~PHMETER_RBUF_UPDATE);
	if(PHMeterDataBuf[PHMETER_ADDR_OFF]!=PHMeterAddr){
		PHMeterErrorHandle();
	}
	if(PHMeterDataBuf[PHMETER_FC_OFF]!=PHMETER_FC){
		PHMeterErrorHandle();
	}
	switch (Temp){
		case PHMETER_PH_PEND:
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_PH_DATALEN){
				PHMeterErrorHandle();
			}
			if(PHMeterDataBuf[PHMETER_SCRCLEN_OFF]!=CRC16(PHMeterDataBuf,PHMETER_SCRCLEN_OFF)){
				PHMeterErrorHandle();
			}
			PHData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_PH_PEND);
			break;
		case PHMETER_T_PEND:
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_T_DATALEN){
				PHMeterErrorHandle();
			}
			if(PHMeterDataBuf[PHMETER_SCRCLEN_OFF]!=CRC16(PHMeterDataBuf,PHMETER_SCRCLEN_OFF)){
				PHMeterErrorHandle();
			}
			TData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_T_PEND);
			break;
		case PHMETER_PHT_PEND:
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_PHT_DATALEN){
				PHMeterErrorHandle();
			}
			if(PHMeterDataBuf[PHMETER_DCRCLEN_OFF]!=CRC16(PHMeterDataBuf,PHMETER_DCRCLEN_OFF)){
				PHMeterErrorHandle();
			}
			PHData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			TData=PHMeterDataBuf[PHMETER_DATA_OFF+2]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+3];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_PH_PEND);
			PHMeterReg&=(~PHMETER_T_PEND);
			break;
		case PHMETER_ORP_PEND:
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_ORP_DATALEN){
				PHMeterErrorHandle();
			}
			if(PHMeterDataBuf[PHMETER_SCRCLEN_OFF]!=CRC16(PHMeterDataBuf,PHMETER_SCRCLEN_OFF)){
				PHMeterErrorHandle();
			}
			ORPData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_ORP_PEND);
			break;
		case PHMETER_ORPT_PEND:
			if(PHMeterDataBuf[PHMETER_DATALEN_OFF]!=PHMETER_TORP_DATALEN){
				PHMeterErrorHandle();
			}
			if(PHMeterDataBuf[PHMETER_DCRCLEN_OFF]!=CRC16(PHMeterDataBuf,PHMETER_DCRCLEN_OFF)){
				PHMeterErrorHandle();
			}
			TData=PHMeterDataBuf[PHMETER_DATA_OFF]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+1];
			ORPData=PHMeterDataBuf[PHMETER_DATA_OFF+2]<<8|PHMeterDataBuf[PHMETER_DATA_OFF+3];
			PHMeterReg&=(~PHMETER_RBUF_UPDATE);
			PHMeterReg&=(~PHMETER_ORP_PEND);
			PHMeterReg&=(~PHMETER_T_PEND);
			break;
		default:	;
	}
}

void PHMeterCheck(void){
	if(PHMeterReg&PHMETER_RBUF_UPDATE){
		PHMeterReceiveHandle();
		PHMeterReg&=~PHMETER_RBUF_UPDATE;
	}
	
	
}












