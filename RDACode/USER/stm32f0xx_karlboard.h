/**
  ******************************************************************************
  * @file    stm32f0xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    04-November-2016
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32F0XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0XX_KARLBOARD_H
#define __STM32F0XX_KARLBOARD_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32F0XX_NUCLEO STM32F0XX-NUCLEO
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
   

/** @defgroup STM32F0XX_NUCLEO_Exported_Types Exported Types
  * @{
  */ 
typedef enum 
{
  LED1 = 0,
	LED2,
	LED3,
} Led_TypeDef;

typedef enum 
{  
  USER_BUTTON1 = 0,
	USER_BUTTON2,
  /* Alias */
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

typedef enum 
{ 
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
} JOYState_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32F0XX_NUCLEO_Exported_Constants Exported Constants
  * @{
  */ 

/** 
* @brief	Define for STM32F0XX_NUCLEO board  
  */ 
#if !defined (USE_STM32F0XX_NUCLEO)
 #define USE_STM32F0XX_NUCLEO
#endif

/** @defgroup STM32F0XX_NUCLEO_LED LED Constants
  * @{
  */
#define LEDn                               3

#define LED1_PIN                           GPIO_PIN_8
#define LED1_GPIO_PORT                     GPIOA
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define LED2_PIN                           GPIO_PIN_6
#define LED2_GPIO_PORT                     GPIOF
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

#define LED3_PIN                           GPIO_PIN_7
#define LED3_GPIO_PORT                     GPIOF
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()  
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE();else if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE();else if((__INDEX__) == 2) LED3_GPIO_CLK_ENABLE();} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE();else if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE();else if((__INDEX__) == 2) LED3_GPIO_CLK_DISABLE();} while(0)

/**
  * @}
  */ 
  
/** @defgroup STM32F0XX_NUCLEO_BUTTON BUTTON Constants
  * @{
  */  
#define BUTTONn                            4

/**
  * @brief User push-button
  */
#define USER_BUTTON1_PIN                         GPIO_PIN_12
#define USER_BUTTON1_GPIO_PORT                   GPIOB
#define USER_BUTTON1_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()   
#define USER_BUTTON1_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()  
#define USER_BUTTON1_EXTI_LINE                   GPIO_PIN_12
#define USER_BUTTON1_EXTI_IRQn                   EXTI4_15_IRQn

#define USER_BUTTON2_PIN                         GPIO_PIN_13
#define USER_BUTTON2_GPIO_PORT                   GPIOB
#define USER_BUTTON2_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()   
#define USER_BUTTON2_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()  
#define USER_BUTTON2_EXTI_LINE                   GPIO_PIN_13
#define USER_BUTTON2_EXTI_IRQn                   EXTI4_15_IRQn

#define USER_BUTTON1_PRES		1		//KEY0按下
#define USER_BUTTON2_PRES		2		//KEY1按下

//#define USER_BUTTON1_STATE  PBin(12)   	//PB12
//#define USER_BUTTON2_STATE  PBin(13)	 	//PB13

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) USER_BUTTON1_GPIO_CLK_ENABLE();else if((__INDEX__) == 1) USER_BUTTON2_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON1_GPIO_CLK_DISABLE() : USER_BUTTON2_GPIO_CLK_DISABLE() )
/**
  * @}
  */ 

/** @defgroup STM32F0XX_NUCLEO_BUS BUS Constants
  * @{
  */ 
/*###################### SPI1 ###################################*/
#define NUCLEO_SPIx                                 SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                          GPIO_AF0_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                   GPIOB
#define NUCLEO_SPIx_SCK_PIN                         GPIO_PIN_3
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                    GPIO_AF0_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT             GPIOB
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                        GPIO_PIN_4
#define NUCLEO_SPIx_MOSI_PIN                        GPIO_PIN_5
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000
/*###################### IIC ###################################*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */
//#define MASTER_BOARD
#define I2C_ADDRESS        0x30F

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 48 MHz */
/* This example use TIMING to 0x00A51314 to reach 1 MHz speed (Rise time = 100 ns, Fall time = 100 ns) */
#define I2C_TIMING      0x00A51314

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C1
#define RCC_I2CxCLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_SYSCLK
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF1_I2C1

/* Definition for I2Cx's NVIC */
#define I2Cx_IRQn                       I2C1_IRQn
#define I2Cx_IRQHandler                 I2C1_IRQHandler

/* Size of Transmission buffer */
#define I2Cx_TXBUFFERSIZE               64
/* Size of Reception buffer */
#define I2Cx_RXBUFFERSIZE               I2Cx_TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

/*
#define NUCLEO_II2C1                                 I2C1
#define NUCLEO_I2C1_CLK_ENABLE()                  __HAL_RCC_I2C1_CLK_ENABLE()

#define NUCLEO_I2C1_SCL_AF                          GPIO_AF1_I2C1
#define NUCLEO_I2C1_SCL_GPIO_PORT                   GPIOB
#define NUCLEO_I2C1_SCL_PIN                         GPIO_PIN_8
#define NUCLEO_I2C1_SCL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_I2C1_SCL_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()

#define NUCLEO_I2C1_SDA_AF                   		GPIO_AF1_I2C1
#define NUCLEO_I2C1_SDA_GPIO_PORT             		GPIOB
#define NUCLEO_I2C1_SDA_GPIO_CLK_ENABLE()   		__HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_I2C1_SDA_GPIO_CLK_DISABLE()  		__HAL_RCC_GPIOB_CLK_DISABLE()
#define NUCLEO_I2C1_SDA_PIN                        	GPIO_PIN_9	*/
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
//#define NUCLEO_I2C1_TIMEOUT_MAX                   	1000

/**
  * @brief  FLASH Control Lines management
  */
#define FLASH_CS_LOW()    HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()   HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  NRF Control Lines management
  */
#define NRF_CS_LOW()    HAL_GPIO_WritePin(NRF_CS_GPIO_PORT, NRF_CS_PIN, GPIO_PIN_RESET)
#define NRF_CS_HIGH()   HAL_GPIO_WritePin(NRF_CS_GPIO_PORT, NRF_CS_PIN, GPIO_PIN_SET)
#define NRF_CE_LOW()    HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_RESET)
#define NRF_CE_HIGH()   HAL_GPIO_WritePin(NRF_CE_GPIO_PORT, NRF_CE_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
      
/**
  * @brief  FLASH Control Interface pins (shield D4)
  */
#define FLASH_CS_PIN                               GPIO_PIN_13
#define FLASH_CS_GPIO_PORT                    	   GPIOC
#define FLASH_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define FLASH_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOC_CLK_DISABLE()
             
/**
  * @brief  NRF Control Interface pins (shield D4)
  */
#define NRF_CS_PIN                              	 GPIO_PIN_12
#define NRF_CS_GPIO_PORT                    	  	 GPIOA
#define NRF_CS_GPIO_CLK_ENABLE()                	 __HAL_RCC_GPIOA_CLK_ENABLE()
#define NRF_CS_GPIO_CLK_DISABLE()               	 __HAL_RCC_GPIOA_CLK_DISABLE()
#define NRF_CE_PIN                              	 GPIO_PIN_11
#define NRF_CE_GPIO_PORT                    	  	 GPIOA
#define NRF_CE_GPIO_CLK_ENABLE()                	 __HAL_RCC_GPIOA_CLK_ENABLE()
#define NRF_CE_GPIO_CLK_DISABLE()               	 __HAL_RCC_GPIOA_CLK_DISABLE()
#define NRF_IRQ_PIN                              	 GPIO_PIN_6
#define NRF_IRQ_GPIO_PORT                    	  	 GPIOB
#define NRF_IRQ_GPIO_CLK_ENABLE()                	 __HAL_RCC_GPIOB_CLK_ENABLE()
#define NRF_IRQ_GPIO_CLK_DISABLE()               	 __HAL_RCC_GPIOB_CLK_DISABLE()

#define NRF_TX_ADR_WIDTH                           5
#define NRF_RX_ADR_WIDTH                           5
#define NRF_TX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
#define NRF_RX_PLOAD_WIDTH  32  	//32字节的用户数据宽度

//NRF24L01寄存器操作命令
#define NRF_READ_REG    0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留

/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_5
#define SD_CS_GPIO_PORT                           GPIOB
#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()

/**
  * @brief  LCD Control Interface pins (shield D10)
  */
#define LCD_CS_PIN                                 GPIO_PIN_6
#define LCD_CS_GPIO_PORT                           GPIOB
#define LCD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()
    
/**
  * @brief  LCD Data/Command Interface pins
  */
#define LCD_DC_PIN                                 GPIO_PIN_9
#define LCD_DC_GPIO_PORT                           GPIOA
#define LCD_DC_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()

/*##################### ADC1 ###################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
#define NUCLEO_ADCx                                 ADC1
#define NUCLEO_ADCx_CLK_ENABLE()                  __HAL_RCC_ADC1_CLK_ENABLE()
#define NUCLEO_ADCx_CLK_DISABLE()                 __HAL_RCC_ADC1_CLK_DISABLE()

#define NUCLEO_ADCx_GPIO_PORT                       GPIOB
#define NUCLEO_ADCx_GPIO_PIN                        GPIO_PIN_0
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()

//KARL
#define NUCLEO_ADC4                                 ADC4
#define NUCLEO_ADC4_CLK_ENABLE()                  __HAL_RCC_ADC4_CLK_ENABLE()
#define NUCLEO_ADC4_CLK_DISABLE()                 __HAL_RCC_ADC4_CLK_DISABLE()

#define NUCLEO_ADC4_GPIO_PORT                       GPIOA
#define NUCLEO_ADC4_GPIO_PIN                        GPIO_PIN_4
#define NUCLEO_ADC4_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_ADC4_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_ADC5                                 ADC5
#define NUCLEO_ADC5_CLK_ENABLE()                  __HAL_RCC_ADC5_CLK_ENABLE()
#define NUCLEO_ADC5_CLK_DISABLE()                 __HAL_RCC_ADC5_CLK_DISABLE()

#define NUCLEO_ADC5_GPIO_PORT                       GPIOA
#define NUCLEO_ADC5_GPIO_PIN                        GPIO_PIN_5
#define NUCLEO_ADC5_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_ADC5_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_ADC6                                 ADC6
#define NUCLEO_ADC6_CLK_ENABLE()                  __HAL_RCC_ADC6_CLK_ENABLE()
#define NUCLEO_ADC6_CLK_DISABLE()                 __HAL_RCC_ADC6_CLK_DISABLE()

#define NUCLEO_ADC6_GPIO_PORT                       GPIOA
#define NUCLEO_ADC6_GPIO_PIN                        GPIO_PIN_6
#define NUCLEO_ADC6_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_ADC6_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_ADC7                                 ADC7
#define NUCLEO_ADC7_CLK_ENABLE()                  __HAL_RCC_ADC7_CLK_ENABLE()
#define NUCLEO_ADC7_CLK_DISABLE()                 __HAL_RCC_ADC7_CLK_DISABLE()

#define NUCLEO_ADC7_GPIO_PORT                       GPIOA
#define NUCLEO_ADC7_GPIO_PIN                        GPIO_PIN_7
#define NUCLEO_ADC7_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_ADC7_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOA_CLK_DISABLE()

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F0XX_NUCLEO_Exported_Functions Exported Functions
  * @{
  */
uint32_t  BSP_GetVersion(void);
/** @defgroup STM32F0XX_NUCLEO_LED_Functions  LED Functions
  * @{
  */ 
void      BSP_LED_Init(Led_TypeDef Led);
void      BSP_LED_DeInit(Led_TypeDef Led);
void      BSP_LED_On(Led_TypeDef Led);
void      BSP_LED_Off(Led_TypeDef Led);
void      BSP_LED_Toggle(Led_TypeDef Led);
/**
  * @}
  */ 
void NRF_IO_Init(void);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);
/** @addtogroup STM32F0XX_NUCLEO_BUTTON_Functions
  * @{
  */                
void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void      BSP_PB_DeInit(Button_TypeDef Button);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);
void 			BSP_PB_Check(void);
#if defined(HAL_ADC_MODULE_ENABLED)
uint8_t          BSP_JOY_Init(void);
JOYState_TypeDef BSP_JOY_GetState(void);
void             BSP_JOY_DeInit(void);
#endif /* HAL_ADC_MODULE_ENABLED */


/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0XX_NUCLEO_H */

    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
