#include "delay.h"
#include "sys.h"
#include "stm32f0xx_hal.h"

					    

//延时nus
//nus为要延时的us数.	
//注意:nus的值,不要大于798915us(最大值即2^24/fac_us@fac_us=21)
void delay_us(u32 nus)
{		
	HAL_Delay(nus);
}

//延时nms 
//nms:0~65535
void delay_ms(u16 nms)
{	 
		HAL_Delay(nms);
}




































