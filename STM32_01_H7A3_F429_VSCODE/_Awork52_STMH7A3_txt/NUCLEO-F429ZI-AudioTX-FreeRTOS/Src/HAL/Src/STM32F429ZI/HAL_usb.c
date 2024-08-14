/**
  ******************************************************************************
  * @file           : USB_DEVICE  
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device 
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx_hal.h"
//#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_def.h"
#if USE_USB_AUDIO_PLAYBACK || USE_USB_AUDIO_RECORDING
#include "usbd_audio.h"
#include "usbd_audio_if.h"
#else
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#endif
#include "InterfUsb.h"
#include "circular_buffers.h"
#include "HAL_usb.h"
#include "usb_uart_rx.h"

#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

/* init function */                        
static void MX_USB_DEVICE_Init(void(*rx_callback)(uint8_t *ptr, size_t len))
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin   = USB_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_OverCurrent_Pin */
    GPIO_InitStruct.Pin   = USB_OverCurrent_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);


    /* Init Device Library,Add Supported Class and Start the library*/
    USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);

#if USE_USB_AUDIO_PLAYBACK || USE_USB_AUDIO_RECORDING
    USBD_RegisterClass(&hUsbDeviceFS, USBD_AUDIO_CLASS);

    /* Add Interface callbacks for AUDIO Class */
    USBD_AUDIO_RegisterInterface(&hUsbDeviceFS, &audio_class_interface);

    /* Start Device Process */
    USBD_Start(&hUsbDeviceFS);
#else
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    CDC_set_rx_callback(rx_callback);

    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);

    USBD_Start(&hUsbDeviceFS);
#endif

    /*Interrupt enable*/
}

/* deinit function */
static void MX_USB_DEVICE_DeInit(void)
{
    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

static bool isVbusHigh(void)
{
    return HAL_GPIO_ReadPin(USB_VBUS_GPIO_Port, USB_VBUS_Pin) == GPIO_PIN_SET;
}

static bool isUsbConfigured(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

#if !USE_USB_AUDIO_PLAYBACK && !USE_USB_AUDIO_RECORDING
static bool isTxBufferEmpty(void)
{
    return CDC_is_txbuffer_empty();
}
#endif

static bool isUsbSuspended(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED);
}

#if !USE_USB_AUDIO_PLAYBACK && !USE_USB_AUDIO_RECORDING
static bool deca_usb_transmit(uint8_t *tx_buffer, int size)
{
    return CDC_Transmit_FS(tx_buffer, size) == USBD_OK;
}
#endif

static void InterfaceUsbUpdate(void)
{

    if(isVbusHigh())
    {//USB port PLUGGED
        if (UsbGetState() == USB_DISCONNECTED)
        {
            Usb.init(usb_rx_callback_circular_buf);
            UsbSetState(USB_CONNECTED);
        }
        else
        if (UsbGetState() == USB_CONNECTED)
        {
            if(isUsbConfigured())
            {
                UsbSetState(USB_CONFIGURED);
            }
            else if(isUsbSuspended())
            {
                Usb.deinit();
                UsbSetState(USB_DISCONNECTED);
            }
        }
    }
    else
    {//USB port UNPLUGGED
        if (UsbGetState() == USB_DISCONNECTED)
        {
            Usb.deinit();
            UsbSetState(USB_DISCONNECTED);
        }
    }
}

/*********************************************************************************/
/** @brief HAL USB API structure
 */
const struct hal_usb_s Usb = {
    .init = &MX_USB_DEVICE_Init,
    .deinit = &MX_USB_DEVICE_DeInit,
#if !USE_USB_AUDIO_PLAYBACK && !USE_USB_AUDIO_RECORDING
    .transmit = &deca_usb_transmit,
#else
    .transmit = NULL,
#endif
    .receive = NULL,
    .update = &InterfaceUsbUpdate,
#if !USE_USB_AUDIO_PLAYBACK && !USE_USB_AUDIO_RECORDING
    .isTxBufferEmpty = &isTxBufferEmpty
#else
    .isTxBufferEmpty = NULL
#endif
};