#include "main.h"
#include "rs485.h"
#include "PHMeter.h"
#include "DissolvedOxygenMeter.h"
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
void CMDSwitch(uint8_t roundCnt);
void DisplaySwitch(uint8_t roundCnt);
/* Private functions ---------------------------------------------------------*/

/* Buffer used for transmission */
extern uint8_t aTxBuffer[];

/* Buffer used for reception */
extern uint8_t aRxBuffer[];	
extern uint8_t aRxBufferBackUp[];


int main(void)
{ 
	uint8_t roundCnt=0x00;
  HAL_Init();
  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  RS485_Init(9600);
	OLED_Init();	
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	OLED_Clear(); 
	DOMeterDisplay();
	RS485_Receive_Data(aRxBuffer,RXBUFFERSIZE);
  while (1)
  {	
		CMDSwitch(roundCnt);
		DisplaySwitch(roundCnt);
		RS485_Check();
		PHMeterCheck();
		DOMeterCheck();	
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    /* Insert delay 100 ms */
    HAL_Delay(100);
	roundCnt++;
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

void CMDSwitch(uint8_t roundCnt){
	uint8_t cntPerRound=0xff;
	if(roundCnt%cntPerRound==0){
				DOMeterRequestData();			
			HAL_Delay(8000);
		}else if(roundCnt%cntPerRound==10){
			PHMeterRequestT();		
			HAL_Delay(1000);
		}else if(roundCnt%cntPerRound==20){
			PHMeterRequestORP();		
			HAL_Delay(1000);
		}else if(roundCnt%cntPerRound==100){
			PHMeterRequestPH();					
			HAL_Delay(1000);
		}
}

void DisplaySwitch(uint8_t roundCnt){
	if(roundCnt%100==0){
			OLED_Clear();
			PHMeterDisplay();
		}else if(roundCnt%100==50){
			OLED_Clear();
			DOMeterDisplay();
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
