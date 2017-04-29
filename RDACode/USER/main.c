#include "main.h"
#include "rs485.h"
#include "PHMeter.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//static GPIO_InitTypeDef  GPIO_InitStruct;
 I2C_HandleTypeDef I2cHandle;
 __IO uint32_t UserButton1Status = 0;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
/* Private functions ---------------------------------------------------------*/

/* Buffer used for transmission */
extern uint8_t aTxBuffer[];

/* Buffer used for reception */
extern uint8_t aRxBuffer[];	
extern uint8_t aRxBufferBackUp[];


int main(void)
{ 
	uint8_t mode;		 
	uint8_t tmp_buf[33];	
	uint16_t t=0;
	mode=0;		//NRF 0：RX	1：TX
	/* This sample code shows how to use GPIO HAL API to toggle LED2 IOs
    in an infinite loop. */

  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  RS485_Init(9600);
	OLED_Init();			//初始化OLED 
	NRF_IO_Init();
	BSP_PB_Init(USER_BUTTON1, BUTTON_MODE_GPIO);
	BSP_PB_Init(USER_BUTTON2, BUTTON_MODE_GPIO);
  /* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
//  LED2_GPIO_CLK_ENABLE();
  /* -2- Configure IOs in output push-pull mode to drive external LEDs */
/*  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = LED2_PIN;
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
*/	
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
		//delay_init(168);	    	 //延时函数初始化	  
	//	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级 
	//	I2C_Init(); 
	while(NRF24L01_Check())
	{
		OLED_ShowString(0,6,"NRF ERR",16);
	}
		OLED_Clear(); 
//	  OLED_On();
/*		OLED_ShowCHinese(0,0,0);//中
		OLED_ShowCHinese(18,0,1);//景
		OLED_ShowCHinese(36,0,2);//园
		OLED_ShowCHinese(54,0,3);//电
		OLED_ShowCHinese(72,0,4);//子
		OLED_ShowCHinese(90,0,5);//科
		OLED_ShowCHinese(108,0,6);//技 
		OLED_ShowString(0,6,"ASCII:",16);  
		OLED_ShowString(63,6,"CODE:",16);  
		OLED_ShowChar(48,6,t,16);//显示ASCII字符	   
		OLED_ShowString(0,180,aTxBuffer,sizeof(aTxBuffer));
		*/
		PHMeterDisplay();
	if(mode==0)//RX模式
	{
		NRF24L01_RX_Mode();		
		while(1)
		{	  		    		    				 
			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
			{
				tmp_buf[32]=0;//加入字符串结束符				
				OLED_ShowString(0,6,"NRF RX",16); 
				OLED_ShowString(2,6,tmp_buf,16);    
			}else HAL_Delay(1);;	   
			t++;
			if(t==1000)//大约1s钟改变一次状态
			{
				t=0;
				BSP_LED_Toggle(LED1);
			} 				    
		};	
	}else
	{
		NRF24L01_TX_Mode();
		OLED_ShowString(0,6,"NRF TX",16); 
		mode=' ';//从空格键开始  
		while(1)
		{	  		   				 
			if(NRF24L01_TxPacket(tmp_buf)==TX_OK)
			{
				OLED_ShowString(2,6,"NRF Send Data",16); 
				for(t=0;t<32;t++)
				{
					if(t+mode>('~')){
						tmp_buf[t]=' ';
					}else{
						tmp_buf[t]=t+mode;					
					}
				}
				mode++; 
				if(mode>'~')mode=' '; 
				tmp_buf[32]=0;//加入结束符	
			}
			else{
				OLED_ShowString(2,6,"NRF Send Fail",16); 
			}
			BSP_LED_Toggle(LED1);
			HAL_Delay(1500);
		}
	}
		RS485_Send_Data(aTxBuffer,TXBUFFERSIZE);
		RS485_Receive_Data(aRxBuffer,RXBUFFERSIZE);
  /* -3- Toggle IOs in an infinite loop */
  while (1)
  {	
		RS485_Check();
		PHMeterCheck();
		BSP_PB_Check();		
		PHMeterDisplay();
		while(UserButton1Status == 0)
		{
			BSP_PB_Check();
      /* Toggle LED2*/
      BSP_LED_Toggle(LED1); 
      HAL_Delay(100);
		}
		OLED_ShowString(6,3,aRxBuffer,16);	
		OLED_ShowString(6,6,aRxBufferBackUp,16);
		
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    /* Insert delay 100 ms */
    HAL_Delay(100);
  }
}
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON1_PIN)
  {  
    UserButton1Status = 1;
  }
	if(GPIO_Pin == USER_BUTTON2_PIN)
  {  
//    UserButton1Status = 1;
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
