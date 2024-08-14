/**
 * @file    HAL_RTC.c
 * 
 * @brief   Hardware abstaction layer for RTC
 *
 * @author  Decawave Applications
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

#include "HAL_rtc.h"
#include "HAL_error.h"
#include "deca_error.h"
#include "int_priority.h"
#include "main.h"

/* WKUP timer
 * Tag uses RTC in Discovery and in the Ranging phase.
 * Node uese RTC to counts the Super Frame period.
 * */
#define WKUP_RESOLUTION_NS          (2e9f/32768.0f)

#ifndef RTC_WKUP_CNT_OVFLW

#define RTC_WKUP_CNT_OVFLW          (32768)

#endif

RTC_HandleTypeDef hrtc;

static void (*rtc_callback)(void);

static void rtc_disable_irq(void)
{
	  HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
}

static void rtc_enable_irq(void)
{
	  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

static void rtc_set_priority_irq(void)
{
	  //set the RTC with the priority higher than highestRTOS
	  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, PRIO_RTC_WKUP_IRQn, 0);
}

static uint32_t rtc_get_counter(void)
{
	  return (uint32_t)((hrtc.Instance->SSR) & RTC_SSR_SS);
}

static uint32_t rtc_get_time_elapsed(uint32_t start, uint32_t stop)
{
	  //RTC is counting down, so we do start - stop to get elapsed time
    int32_t tmp = start - stop;

    // check roll over
    if (tmp < 0)
    {
        tmp += 32768;
    }
    return tmp;
}

//-----------------------------------------------------------------------------
/*
 * @brief   RTC callback called by the RTC driver
 * DO NOT rename this function
 * */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *phrtc)
{
    if(phrtc == &hrtc)
    {
        if(rtc_callback)
        {
          rtc_callback();
        }
    }
}

static void rtc_set_callback(void(*cb)(void))
{
    rtc_callback = cb;
}

static float rtc_get_wakup_resolution_ns(void)
{
    return WKUP_RESOLUTION_NS;
}

//-----------------------------------------------------------------------------
/*
 * @brief   setup RTC Wakeup timer
 *          period_ms is awaiting time in ms
 * */
void rtc_configure_wakeup_ms(uint32_t period_ms)
{
    uint32_t period_ns = (1e6 * period_ms)/WKUP_RESOLUTION_NS;

    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, period_ns, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
    {
      error_handler(1,_ERR_Configure_WKUP_Timer);
    }
}

//-----------------------------------------------------------------------------
/*
 * @brief   setup RTC Wakeup timer
 *          period_ns is awaiting time in ns
 * */
void rtc_configure_wakeup_ns(uint32_t period_ns)
{
    period_ns = (uint32_t)(period_ns/ WKUP_RESOLUTION_NS);

    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, period_ns, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
    {
      error_handler(1,_ERR_Configure_WKUP_Timer);
    }
}

/** @brief Initialization of the RTC driver instance
 */
static void rtc_init(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

      /**Initialize RTC Only
      */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 1;
    hrtc.Init.SynchPrediv = 32767;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
      Error_Handler();
    }

      /**Initialize RTC and set the Time and Date
      */
    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 0x0;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }

      HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
    }
      /**Enable the WakeUp
      */
    if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
    {
      Error_Handler();
    }
}

static void rtc_deinit(void)
{
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
	rtc_callback = NULL;
}

/*********************************************************************************/
/** @brief HAL RTC API structure
 */
const struct hal_rtc_s Rtc = {
	.init = &rtc_init,
	.deinit = &rtc_deinit,
	.enableIRQ = &rtc_enable_irq,
	.disableIRQ = &rtc_disable_irq,
	.setPriorityIRQ = &rtc_set_priority_irq,
	.getTimestamp = &rtc_get_counter,
	.getTimeElapsed = &rtc_get_time_elapsed,
	.reload = NULL,
	.configureWakeup_ms = &rtc_configure_wakeup_ms,
	.configureWakeup_ns = &rtc_configure_wakeup_ns,
  .getWakeupResolution_ns = &rtc_get_wakup_resolution_ns,
	.setCallback = &rtc_set_callback
};
