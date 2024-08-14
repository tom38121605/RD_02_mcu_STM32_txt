/**
 * @file      NUCLEO-F429ZI.c
 * 
 * @brief     Board specific initialization
 *
 * @author    Decawave Applications
 *
 * @attention Copyright (c) 2021 - 2022, Qorvo US, Inc.
 * All rights reserved
 * Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions, and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 3. You may only use this software, with or without any modification, with an
 *  integrated circuit developed by Qorvo US, Inc. or any of its affiliates
 *  (collectively, "Qorvo"), or any module that contains such integrated circuit.
 * 4. You may not reverse engineer, disassemble, decompile, decode, adapt, or
 *  otherwise attempt to derive or gain access to the source code to any software
 *  distributed under this license in binary or object code form, in whole or in
 *  part.
 * 5. You may not use any Qorvo name, trademarks, service marks, trade dress,
 *  logos, trade names, or other symbols or insignia identifying the source of
 *  Qorvo's products or services, or the names of any of Qorvo's developers to
 *  endorse or promote products derived from this software without specific prior
 *  written permission from Qorvo US, Inc. You must not call products derived from
 *  this software "Qorvo", you must not have "Qorvo" appear in their name, without
 *  the prior permission from Qorvo US, Inc.
 * 6. Qorvo may publish revised or new version of this license from time to time.
 *  No one other than Qorvo US, Inc. has the right to modify the terms applicable
 *  to the software provided under this license.
 * THIS SOFTWARE IS PROVIDED BY QORVO US, INC. "AS IS" AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. NEITHER
 *  QORVO, NOR ANY PERSON ASSOCIATED WITH QORVO MAKES ANY WARRANTY OR
 *  REPRESENTATION WITH RESPECT TO THE COMPLETENESS, SECURITY, RELIABILITY, OR
 *  ACCURACY OF THE SOFTWARE, THAT IT IS ERROR FREE OR THAT ANY DEFECTS WILL BE
 *  CORRECTED, OR THAT THE SOFTWARE WILL OTHERWISE MEET YOUR NEEDS OR EXPECTATIONS.
 * IN NO EVENT SHALL QORVO OR ANYBODY ASSOCIATED WITH QORVO BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 */


//#include "thisBoard.h"
//#include "boards.h"
#include "HAL_rtc.h"
#include "HAL_uwb.h"
#include "HAL_watchdog.h"
#include "crc16.h"
#include "custom_board.h"
#include "deca_device_api.h"
#include "deca_error.h"
#include "int_priority.h"
#include "stm32f4xx_hal.h"

#include "HAL_rtc.h"
#include "HAL_uwb.h"
#include "HAL_watchdog.h"
#include "rf_tuning_config.h"
#include "appConfig.h"

#include "HAL_usb.h"

#if USE_AUDIO
#include "user_audio_config.h"
#if USE_AUDIO_RX
#include "audio_rx.h"
#elif USE_AUDIO_TX
#include "audio_tx.h"
#endif
#endif

/* TODO: this variable is of no use and can be removed */
static struct dwchip_s *old_dw = NULL;

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi5_rx;
DMA_HandleTypeDef hdma_spi5_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

I2S_HandleTypeDef hi2s;
DMA_HandleTypeDef hdma_i2s_tx;

static int g_network_is_up;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(int);
static void MX_TIM5_Init(void);
#if USE_AUDIO
static void MX_TIM4_Init(void);
#if USE_I2S_AUDIO_PLAYBACK
static void MX_I2S3_Init(void);
#endif
#endif
static void MX_USART3_UART_Init(void);

static void MX_RNG_Init(void);
void StartDefaultTask(void const * argument);
static void MX_NVIC_Init(void);

extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

extern void mcps_set_uptime_us(uint32_t time_us);
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }

  if (htim->Instance == TIM3)
  {
      HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);
  }
/* USER CODE BEGIN Callback 1 */
#if USE_AUDIO
  if (htim->Instance == TIM4)
  {
#if USE_AUDIO_RX
#if USE_USB_AUDIO_RECORDING
      audio_rx_data_out();
#else
      audio_rx_debug_output();
#endif
#endif
#if USE_AUDIO_TX
      audio_tx_debug_output();
#endif
  }
#endif

  if (htim->Instance == TIM5)
  {
    mcps_set_uptime_us(65535 + 1);
  }
/* USER CODE END Callback 1 */
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
       _Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_RST_GPIO_Port, DW_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DW_WUP_GPIO_Port, DW_WUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : DW_IRQ_Pin */
  GPIO_InitStruct.Pin = DW_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DW_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_RST_Pin */
  GPIO_InitStruct.Pin = DW_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DW_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_CS_Pin */
  GPIO_InitStruct.Pin = DW_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DW_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_WAKEUP_Pin */
  GPIO_InitStruct.Pin = DW_WUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DW_WUP_GPIO_Port, &GPIO_InitStruct);
  
//  {/* USB: disconnect DP pin */
//   /* USB: all usb pins will be initialized in the MX_USB_DEVICE_Init() -> HAL_PCD_MspInit() */
//      GPIO_InitStruct.Pin   = USB_DP_Pin;
//      GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//      GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//      HAL_GPIO_Init(USB_DP_GPIO_Port, &GPIO_InitStruct);
//  }

  {/* SWD: it is mandatory to configure Alternate function when the AF mode is selected */
      GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14;
      GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*BLUE LED Init on Anchor board*/
  GPIO_InitStruct.Pin = BLUE_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_PORT, &GPIO_InitStruct);

  /*GREEN LED Init on Anchor board*/
  GPIO_InitStruct.Pin = GREEN_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_PORT, &GPIO_InitStruct);

  /*RED LED Init on Anchor board*/
  GPIO_InitStruct.Pin = RED_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_PORT, &GPIO_InitStruct);

  /*USER Button on Anchor board*/
  GPIO_InitStruct.Pin = USER_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_BUTTON_PORT, &GPIO_InitStruct);
}

static void MX_TIM3_Init(int network_status)
{
    HAL_NVIC_SetPriority(TIM3_IRQn, PRIO_TIM3_IRQn ,0);
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;

    htim3.Init.Period = 10000;

    if(network_status == 0)
    {
        htim3.Init.Prescaler = 540;
    }

    else if(network_status == 1)
    {
        htim3.Init.Prescaler = 2000;
    }
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if(HAL_TIM_Base_Init(&htim3) == HAL_OK)
    {
      /* TIM time Base Initialisation is successful*/
    }

    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_TIM_Base_Start_IT(&htim3);
}

#if USE_AUDIO
#if USE_AUDIO_TX
#define TIM_SampleSyncPeriod 10000
#define TIM_SampleSyncPrescaler 10000
#elif USE_AUDIO_RX
#if USE_USB_AUDIO_RECORDING
#define TIM_SampleSyncPeriod 1000
#define TIM_SampleSyncPrescaler 1000000
#else
#define TIM_SampleSyncPeriod 10000
#define TIM_SampleSyncPrescaler 10000
#endif
#endif
#define TIM_SampleSyncIRQPriority 7

static void MX_TIM4_Init()
{
    HAL_NVIC_SetPriority(TIM4_IRQn, TIM_SampleSyncIRQPriority, 0);
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;

    /* Compute the prescaler value to have TIM_SampleSync counter clock equal to 10000 Hz */
    uint32_t ssPrescalerValue = (uint32_t)((SystemCoreClock / 2) / TIM_SampleSyncPrescaler) - 1;

    /* Initialize TIM_SampleSync peripheral as follows:
         + Period = 1000 - 1
         + Prescaler = ((SystemCoreClock / 2)/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
    */
    htim4.Init.Period    = TIM_SampleSyncPeriod - 1;
    htim4.Init.Prescaler = ssPrescalerValue;
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.RepetitionCounter = TIM_OPMODE_REPETITIVE;
    if (HAL_TIM_Base_Init(&htim4) == HAL_OK)
    {
        /* TIM time Base Initialisation is successful*/
    }

    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    HAL_TIM_Base_Start_IT(&htim4);
}

#if USE_I2S_AUDIO_PLAYBACK
/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s.Instance = SPI3;
  hi2s.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s.Init.AudioFreq = I2S_AUDIOFREQ_HZ;
  hi2s.Init.DataFormat = I2S_DATAFORMAT_BIT;
  hi2s.Init.CPOL = I2S_CPOL_LOW;
  hi2s.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}
#endif
#endif

static void MX_TIM5_Init(void)
{
    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
    __HAL_RCC_TIM5_CLK_ENABLE();
 
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = (72 - 1);
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 65535;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.RepetitionCounter = TIM_OPMODE_REPETITIVE;
    if (HAL_TIM_Base_Init(&htim5) == HAL_OK)
    {
        /* TIM time Base Initialisation is successful*/
    }

    HAL_NVIC_EnableIRQ(TIM5_IRQn);

    HAL_TIM_Base_Start_IT(&htim5);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, PRIO_USART3_IRQn, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, PRIO_DMA1_Stream1_IRQn, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn); //USART3 RX

  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, PRIO_DMA1_Stream3_IRQn, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn); //USART3 TX

#if USE_I2S_AUDIO_PLAYBACK
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, PRIO_DMA1_Stream5_IRQn, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);  //I2S3 TX
#endif

  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, PRIO_SPI1_IRQn, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, PRIO_DMA2_Stream2_IRQn, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, PRIO_DMA2_Stream3_IRQn, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

//  /* ETH_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(ETH_IRQn, PRIO_ETH_IRQn, 0);
//  HAL_NVIC_EnableIRQ(ETH_IRQn);

  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, PRIO_TIM1_CC_IRQn, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


  HAL_RCC_GetSysClockFreq();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
#if USE_I2S_AUDIO_PLAYBACK
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = I2S_PLL_DEFAULT_NUM_N;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = I2S_PLL_DEFAULT_NUM_R;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/* @fn  peripherals_init
 *
 * @param[in] void
 * */
void peripherals_init(void)
{

}

void BoardInit(void)
{
    /* USER CODE BEGIN 1 */
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_DBGMCU_EnableDBGStandbyMode();

#ifdef FIRMWARE_UPGRADE
    SCB->VTOR = ADDR_FLASH_SECTOR_6;
#else
    SCB->VTOR = ADDR_FLASH_SECTOR_0;
#endif

#ifdef ENABLE_SEMIHOSTING
    initialise_monitor_handles();
    setbuf(stdout, NULL);
#endif

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */


    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM3_Init(g_network_is_up);
    MX_TIM5_Init();
#if USE_AUDIO
    MX_TIM4_Init();
#if USE_I2S_AUDIO_PLAYBACK
    MX_I2S3_Init();
#endif
#endif
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_RNG_Init();
    Watchdog.init(60000);
    Rtc.init();

    /* Initialize interrupts */
    MX_NVIC_Init();

    Watchdog.refresh();
    
     /* Initialize random number generator needs for LWIP_RAND */
    uint32_t tmp;
    HAL_RNG_GenerateRandomNumber(&hrng, &tmp);
#if 0
    srand(tmp);
#endif
}

void board_interface_init(void)
{
}

int uwb_init(void)
{
    hal_uwb.init();

    hal_uwb.irq_init();
    hal_uwb.disableIRQ();

    hal_uwb.reset();

    int status = hal_uwb.probe();

    /* This section is added to initialize the local in dw_drivers
    Without this initialization The command like xtaltrim will fail*/
    if (status == DWT_SUCCESS)
    {
      old_dw = dwt_update_dw(hal_uwb.uwbs->dw);
      /* This initialization is added to have the correct values of Part ID and Lot ID */
      status = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID);

      dwt_txconfig_t *txConfig = get_dwt_txconfig();

        if(is_auto_restore_bssConfig())
        {
            //we are coming from a au default config, likely due to an initial powerup after FW upgrade
            //update what has to be updated, post chip detection
            rf_tuning_set_tx_power_pg_delay(hal_uwb.uwbs->devid);
            clear_auto_restore_bssConfig();
            save_bssConfig();
        }
      dwt_set_alternative_pulse_shape(1);
      dwt_enable_disable_eq(DWT_EQ_ENABLED);
      dwt_configuretxrf(txConfig);
    }

    return status;
}
