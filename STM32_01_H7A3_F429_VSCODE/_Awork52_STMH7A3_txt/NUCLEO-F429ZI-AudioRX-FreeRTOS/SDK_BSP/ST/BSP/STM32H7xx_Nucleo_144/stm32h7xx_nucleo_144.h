/** 
  ******************************************************************************
  * @file    stm32h7xx_nucleo_144.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32H7XX-Nucleo-144 Kit 
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
#ifndef __STM32H7XX_NUCLEO_144_H
#define __STM32H7XX_NUCLEO_144_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32H7XX_NUCLEO_144
  * @{
  */

/** @addtogroup STM32H7XX_NUCLEO_144_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32H7XX_NUCLEO_144_LOW_LEVEL_Exported_Types STM32H7XX NUCLEO 144 LOW LEVEL Exported Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED_GREEN = LED1,
  LED2 = 1,
  LED_BLUE = LED2,
  LED3 = 2,
  LED_RED = LED3
}Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY = BUTTON_USER
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32H7XX_NUCLEO_144_LOW_LEVEL_Exported_Constants STM32H7XX NUCLEO 144 LOW LEVEL Exported Constants
  * @{
  */ 

/** 
  * @brief Define for STM32H7XX_NUCLEO_144 board  
  */ 
#if !defined (USE_STM32H7XX_NUCLEO_144)
 #define USE_STM32H7XX_NUCLEO_144
#endif

/** @defgroup STM32H7XX_NUCLEO_144_LOW_LEVEL_LED STM32H7XX NUCLEO 144 LOW LEVEL LED
  * @{
  */
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_0
#define LED1_GPIO_PORT                          GPIOB
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LED2_PIN                                GPIO_PIN_1
#define LED2_GPIO_PORT                          GPIOE
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOE_CLK_DISABLE()  

#define LED3_PIN                                GPIO_PIN_14
#define LED3_GPIO_PORT                          GPIOB
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) {__HAL_RCC_GPIOB_CLK_ENABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_ENABLE();   }} while(0)	
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) {__HAL_RCC_GPIOB_CLK_DISABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_DISABLE();   }} while(0)	
/**
  * @}
  */ 
  
/** @defgroup STM32H7XX_NUCLEO_144_LOW_LEVEL_BUTTON STM32H7XX NUCLEO 144 LOW LEVEL BUTTON
  * @{
  */  
#define BUTTONn                                 1  

/**
 * @brief Key push-button
 */
#define USER_BUTTON_PIN                          GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                    GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_LINE                    GPIO_PIN_13
#define USER_BUTTON_EXTI_IRQn                    EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)      USER_BUTTON_GPIO_CLK_ENABLE()
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)     USER_BUTTON_GPIO_CLK_DISABLE() 

/* Aliases */
#define KEY_BUTTON_PIN                       USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                 USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()         USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()        USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                 USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                 USER_BUTTON_EXTI_IRQn


/**
  * @brief OTG_FS1 OVER_CURRENT and POWER_SWITCH Pins definition
  */


#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_7
#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

/**
  * @}
  */

/** @defgroup STM32H7XX_NUCLEO_144_LOW_LEVEL_Exported_Macros STM32H7XX NUCLEO 144 LOW LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup STM32H7XX_NUCLEO_144_LOW_LEVEL_Exported_Functions STM32H7XX NUCLEO 144 LOW LEVEL Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);

  
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

#endif /* __STM32H7XX_NUCLEO_144_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
