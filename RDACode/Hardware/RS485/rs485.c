#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "string.h"
#include "PHMeter.h"
#include "DissolvedOxygenMeter.h"
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
/**
  * @brief Init RS485.
  * @param  bound:BaudRate 
  *   This parameter can be one of the following values:
  * @arg 9600 115200 etc.
  * @note None
  * @retval None
  */
	
void RS485_Init(u32 bound)
{  
	GPIO_InitTypeDef  gpioinitstruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
	gpioinitstruct.Pin = RS485_2_RE_PIN;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH; 
  HAL_GPIO_Init(RS485_2_RE_GPIO_PORT, &gpioinitstruct);
	
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
/**
  * @brief Send data with RS485.
	* @param  buf: first address of the data buffer to be sent
						len: length of data to be sent. in Byte
  * @arg 
  * @note None
  * @retval None
  */
void RS485_Send_Data(u8 *buf,u8 len)
{
	//RS485_Receive_Data(aRxBuffer,RXBUFFERSIZE);
	RS485_2_RE_HIGH();
//	HAL_Delay(1000);
	 /* The board sends the message and expects to receive it back */
  
  /*##-2- Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
	if(HAL_UART_Transmit(&UartHandle,(uint8_t*)buf, len,1000) != HAL_OK)
  //if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)buf, len)!= HAL_OK)
  {
    Error_Handler();
  }
	/*##-3- Wait for the end of the transfer ###################################*/   
//  while (UartReady != SET)
//  {
//  }
  
  /* Reset transmission flag */
  UartReady = RESET;
//  RS485_Receive_Data(aRxBuffer,RXBUFFERSIZE);
  RS485_2_RE_LOW();
}
/**
  * @brief Get data received by RS485.
	* @param  buf: first address of the data buffer used to store data received
						len: length of data read. in Byte
  * @arg 
  * @note None
  * @retval None
  */
void RS485_Receive_Data(u8 *buf,u8 len)
{
//	uint8_t t=0x00;
	/*##-4- Put UART peripheral in reception process ###########################*/  
  while(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)buf, len) != HAL_OK)
  {
    Error_Handler();
//		if(t>>7){
//			UartHandle.RxState = HAL_UART_STATE_READY;
//		}
//		t++;
  }
}

void RS485_Check(void){
	uint8_t ReceiveDataAddr=0x00;
	if(RS485Reg){
		memcpy(aRxBufferBackUp,aRxBuffer,RXBUFFERSIZE);
		memcpy(aRxBuffer,allZero,RXBUFFERSIZE);
		if(RS485Reg&RS485_2_REC){
			ReceiveDataAddr=aRxBufferBackUp[0];
			switch (ReceiveDataAddr){
				case PHMeterAddr:
					memcpy(PHMeterDataBuf,aRxBufferBackUp,PHMETER_DATABUF_SIZE);
					PHMeterReg|=PHMETER_RBUF_UPDATE;
					break;
				case DOMeterAddr:
					memcpy(DOMeterDataBuf,aRxBufferBackUp,DOMETER_DATABUF_SIZE);
					DOMeterReg|=DOMETER_RBUF_UPDATE;
					break;
				
				default: ;
			
			}
			memcpy(aRxBufferBackUp,allZero,RXBUFFERSIZE);
		}
	RS485Reg&=~RS485_2_REC;
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
	RS485Reg|=RS485_2_REC;
	RS485_Receive_Data(aRxBuffer,RXBUFFERSIZE);
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





