#include "delay.h"
#include "sys.h"
#include "stm32f0xx_hal.h"

					    

//��ʱnus
//nusΪҪ��ʱ��us��.	
//ע��:nus��ֵ,��Ҫ����798915us(���ֵ��2^24/fac_us@fac_us=21)
void delay_us(u32 nus)
{		
	HAL_Delay(nus);
}

//��ʱnms 
//nms:0~65535
void delay_ms(u16 nms)
{	 
		HAL_Delay(nms);
}




































