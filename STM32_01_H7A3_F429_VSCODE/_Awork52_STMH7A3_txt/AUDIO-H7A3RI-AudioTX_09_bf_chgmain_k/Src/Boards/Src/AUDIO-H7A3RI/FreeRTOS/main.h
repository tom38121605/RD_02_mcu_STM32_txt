/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32h7xx_hal.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#define LWIP_TIMEVAL_PRIVATE 0

#define DW_SCK_Pin GPIO_PIN_10
#define DW_SCK_GPIO_Port GPIOC
#define DW_MISO_Pin GPIO_PIN_11
#define DW_MISO_GPIO_Port GPIOC
#define DW_MOSI_Pin GPIO_PIN_12
#define DW_MOSI_GPIO_Port GPIOC
#define DW_IRQ_Pin GPIO_PIN_3
#define DW_IRQ_GPIO_Port GPIOB
#define DW_RST_Pin GPIO_PIN_2
#define DW_RST_GPIO_Port GPIOD
#define DW_CS_Pin GPIO_PIN_15
#define DW_CS_GPIO_Port GPIOA

// USB
#define USB_DET_Pin GPIO_PIN_6
#define USB_DET_GPIO_Port GPIOC
#define USB_FS_VBUS_Pin USB_DET_Pin

#define USB_FS_VBUS_GPIO_Port USB_DET_GPIO_Port

#define USB_FS_N_Pin GPIO_PIN_11
#define USB_FS_N_GPIO_Port GPIOA
#define USB_FS_P_Pin GPIO_PIN_12
#define USB_FS_P_GPIO_Port GPIOA

#define RED_LED_PORT 	GPIOA
#define RED_LED_PIN		GPIO_PIN_10

#define BLUE_LED_PORT 	GPIOA
#define BLUE_LED_PIN 	GPIO_PIN_9

#define GREEN_LED_PORT 	GPIOA
#define GREEN_LED_PIN 	GPIO_PIN_8

/*External Line Interrupts SPI and chip wake up*/
#define DW_SPI_EXIT_IRQ_NUM EXTI3_IRQn
#define DW_RST_EXIT_IRQ_NUM EXTI2_IRQn

/* USER CODE BEGIN Private defines */
#define DW_WUP_Pin        GPIO_PIN_4
#define DW_WUP_GPIO_Port  GPIOB

// #define USE_DHCP       /* enable DHCP, if disabled static address is used*/
// #define USE_HTTP_SERVER /* enable HTTP server with real-time statistics */

#if USE_I2S_AUDIO_PLAYBACK
#define DSP_PWR_CTRL_Pin GPIO_PIN_5
#define DSP_PWR_CTRL_GPIO_Port GPIOB
#endif

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
