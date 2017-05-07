#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_karlboard.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define RS485_2                           USART2
#define RS485_2_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
#define RS485_2_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define RS485_2_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define RS485_2_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define RS485_2_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define RS485_2_TX_PIN                    GPIO_PIN_2
#define RS485_2_TX_GPIO_PORT              GPIOA
#define RS485_2_TX_AF                     GPIO_AF1_USART2
#define RS485_2_RX_PIN                    GPIO_PIN_3
#define RS485_2_RX_GPIO_PORT              GPIOA
#define RS485_2_RX_AF                     GPIO_AF1_USART2

/**
  * @brief  RS485_1 Control Lines management
  */
#define RS485_1_RE_PIN                           GPIO_PIN_14
#define RS485_1_RE_GPIO_PORT                     GPIOB
#define RS485_1_RE_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define RS485_1_RE_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()
#define RS485_1_RE_LOW()    HAL_GPIO_WritePin(RS485_1_RE_GPIO_PORT, RS485_1_RE_PIN, GPIO_PIN_RESET)
#define RS485_1_RE_HIGH()   HAL_GPIO_WritePin(RS485_1_RE_GPIO_PORT, RS485_1_RE_PIN, GPIO_PIN_SET)

/**
  * @brief  RS485_2 Control Lines management
  */
#define RS485_2_RE_PIN                           GPIO_PIN_1
#define RS485_2_RE_GPIO_PORT                     GPIOA
#define RS485_2_RE_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define RS485_2_RE_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()
#define RS485_2_RE_LOW()    HAL_GPIO_WritePin(RS485_2_RE_GPIO_PORT, RS485_2_RE_PIN, GPIO_PIN_RESET)
#define RS485_2_RE_HIGH()   HAL_GPIO_WritePin(RS485_2_RE_GPIO_PORT, RS485_2_RE_PIN, GPIO_PIN_SET)

/* Definition for USARTx's NVIC */
#define RS485_2_IRQn                      USART2_IRQn
#define RS485_2_IRQHandler                USART2_IRQHandler


/* Size of Trasmission buffer */
#define TXBUFFERSIZE                     64
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


#define RS485_2_REC                      0X01
//如果想串口中断接收，设置EN_USART2_RX为1，否则设置为0
#define EN_USART2_RX 	1			//0,不接收;1,接收.
													 
void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,u8 len);	
void RS485_Check(void);
#endif	   
















