#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "string.h"
#include "PHMeter.h"
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
static void Error_Handler(void);
/* Buffer used for transmission */
uint8_t aTxBuffer[TXBUFFERSIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t allZero[TXBUFFERSIZE];
uint8_t aRxBufferBackUp[RXBUFFERSIZE];
uint8_t RS485Reg=0x00;
#if EN_USART2_RX   		//如果使能了接收   	  
//接收缓存区 	
u8 RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
//接收到的数据长度
u8 RS485_RX_CNT=0;   

#endif										 
//初始化IO 串口2
//bound:波特率	  
void RS485_Init(u32 bound)
{  
	
	UartHandle.Instance        = USART2;

  UartHandle.Init.BaudRate   = bound;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  

}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	 /* The board sends the message and expects to receive it back */
  
  /*##-2- Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)buf, len)!= HAL_OK)
  {
    Error_Handler();
  }
	/*##-3- Wait for the end of the transfer ###################################*/   
  while (UartReady != SET)
  {
  }
  
  /* Reset transmission flag */
  UartReady = RESET;
  
}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(u8 *buf,u8 len)
{
	/*##-4- Put UART peripheral in reception process ###########################*/  
  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)buf, len) != HAL_OK)
  {
    Error_Handler();
  }
}

void RS485_Check(void){
	uint8_t ReceiveDataAddr=0x00;
	if(RS485Reg){
		if(RS485Reg&RS485_2_REC){
			ReceiveDataAddr=aRxBufferBackUp[0];
			switch (ReceiveDataAddr){
				case PHMeterAddr:
					memcpy(PHMeterDataBuf,aRxBufferBackUp,PHMETER_DATABUF_SIZE);
					PHMeterReg|=PHMETER_RBUF_UPDATE;
					break;
				
				
				default: ;
			
			}
			memcpy(aRxBufferBackUp,allZero,RXBUFFERSIZE);
		}
	
	}


}
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;
	
  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;	
	memcpy(aRxBufferBackUp,aRxBuffer,RXBUFFERSIZE);
	memcpy(aRxBuffer,allZero,RXBUFFERSIZE);
  RS485_Receive_Data(aRxBuffer,RXBUFFERSIZE);
	RS485Reg|=RS485_2_REC;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
    while(1)
    {
    /* Error if LED2 is slowly blinking (1 sec. period) */
    BSP_LED_Toggle(LED2); 
    HAL_Delay(1000); 
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif





