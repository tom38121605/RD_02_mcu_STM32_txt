/**
 * @file      audio_i2s_speaker.c
 *
 * @brief     Audio i2s speaker functionalities
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
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "usb_uart_tx.h"  //--add

#if USE_I2S_AUDIO_PLAYBACK
#include "audio_i2s_speaker.h"
#include "common_audio.h"

#if I2S_BUFFER_USE_TASK
#include "create_audio_i2s_task.h"
#include "task_audio_i2s.h"
#define AUDIO_I2S_TASK_STACK_SIZE_BYTES 2048

static task_signal_t audioI2sTask;
static uint8_t * i2s_next_frame_data = NULL;
static uint16_t i2s_next_frame_size = 0;
#endif

// I2S data need to do padding before send to DMA if using 24Bits format
#if AUDIO_RESOLUTION_BYTE == 3
#define AUDIO_I2S_PACKAGE_BYTES 4
#define AUDIO_I2S_PACKAGE_SIZE  (AUDIO_I2S_DATA_SIZE / 3 * 4)
#else
#define AUDIO_I2S_PACKAGE_BYTES 2
#define AUDIO_I2S_PACKAGE_SIZE  (AUDIO_I2S_DATA_SIZE)
#endif

#define AUDIO_I2S_BUFFER_SIZE (AUDIO_I2S_PACKAGE_SIZE * 2)

/* Buffer for I2S DMA */
static uint8_t i2s_data_buffer[AUDIO_I2S_BUFFER_SIZE] = {0};
static uint16_t i2s_data_buffer_size = AUDIO_I2S_BUFFER_SIZE;
/* Switch I2S buffer when DMA halt complete */
static uint16_t i2s_next_frame_index                 = 0;

extern I2S_HandleTypeDef hi2s;
extern DMA_HandleTypeDef hdma_i2s_tx;

/* I2S DMA status*/
typedef enum
{
  AUDIO_I2S_DMA_OFF = 0,
  AUDIO_I2S_DMA_STARTED,
  AUDIO_I2S_DMA_PAUSED,
} audio_i2s_dma_status;

static audio_i2s_dma_status dma_status = AUDIO_I2S_DMA_OFF;

/* For 24Bits audio format, data need padding to 32Bits before send to I2S interface*/
#if AUDIO_RESOLUTION_BYTE == 3
void audio_i2s_data_padding_24to32(uint8_t *data_dest, uint8_t *data_src, int data_size)
{
    int j = 0;
    for (int i = 0; i < data_size; i = i + 3)
    {
        data_dest[j++] = data_src[i + 1]; // Bit2
        data_dest[j++] = data_src[i + 2]; // Bit1
        data_dest[j++] = 0x0;             // Bit4
        data_dest[j++] = data_src[i];     // Bit3
    }
}
#endif

I2S_HandleTypeDef *audio_i2s_get_handle()
{
    return &hi2s;
}

static void audio_i2s_buffer_load_data(uint8_t *data_buffer, uint16_t data_size)
{
#if AUDIO_RESOLUTION_BYTE == 3
    audio_i2s_data_padding_24to32(i2s_data_buffer + i2s_next_frame_index, data_buffer, data_size);
#else
    memcpy(i2s_data_buffer + i2s_next_frame_index, data_buffer, data_size);
#endif

    /* Update the next data index for I2S DMA buffer, switch in DMA interrupt handling */
    if (0 == i2s_next_frame_index)
    {
        i2s_next_frame_index = i2s_data_buffer_size / 2;
    }
    else
    {
        i2s_next_frame_index = 0;
    }
}

/**
 * @description: Fill I2S buffer and start DMA
 * @param {uint8_t} *data_buffer    audio data
 * @param {uint16_t} data_size  should be half of I2S DMA buffer size
 * @return {*}
 */
void audio_i2s_start(uint8_t *data_buffer, uint16_t data_size)
{
    if (dma_status == AUDIO_I2S_DMA_OFF)
    {
//--copy_tx_msg((uint8_t*)"starti2s1\r\n", 11);   //ok
        memset(i2s_data_buffer, 0, sizeof(i2s_data_buffer));
        i2s_next_frame_index = AUDIO_I2S_BUFFER_SIZE / 2;
#if I2S_BUFFER_USE_TASK
        i2s_next_frame_data = data_buffer;
        i2s_next_frame_size = data_size;
        audio_i2s_task_notify(AUDIO_I2S_DATA_NTF);
#else
         audio_i2s_buffer_load_data(data_buffer, data_size);
#endif

        /* Start I2S transmission */
#ifdef STM32F429xx
        // On STM32F429xx, the data length here means the number of 24-bit or 32-bit data length
        HAL_I2S_Transmit_DMA(&hi2s, (uint16_t *)i2s_data_buffer, sizeof(i2s_data_buffer) / AUDIO_I2S_PACKAGE_BYTES);
#else
        // On other STM platform, the data length here means the number of 16-bit data length.
        HAL_I2S_Transmit_DMA(&hi2s, (uint16_t *)i2s_data_buffer, sizeof(i2s_data_buffer) / 2);
#endif
        dma_status = AUDIO_I2S_DMA_STARTED;
    }
    else
    {
//copy_tx_msg((uint8_t*)"starti2s2\r\n", 11);   //no

        audio_i2s_update_buffer(data_buffer, data_size);
        audio_i2s_resume();
    }
}

/**
 * @description: Update I2S DMA buffer when half and all complete
 * @param {uint8_t} *data_buffer
 * @param {uint16_t} data_size
 * @return {*}
 */
void audio_i2s_update_buffer(uint8_t *data_buffer, uint16_t data_size)
{
#if I2S_BUFFER_USE_TASK
    i2s_next_frame_data = data_buffer;
    i2s_next_frame_size = data_size;
    audio_i2s_task_notify(AUDIO_I2S_DATA_NTF);
#else
    audio_i2s_buffer_load_data(data_buffer, data_size);
#endif
}

void audio_i2s_pause()
{
    if(dma_status == AUDIO_I2S_DMA_STARTED)
    {
        HAL_I2S_DMAPause(&hi2s);
        dma_status = AUDIO_I2S_DMA_PAUSED;
    }
}

void audio_i2s_resume()
{
    if(dma_status == AUDIO_I2S_DMA_PAUSED)
    {
        HAL_I2S_DMAResume(&hi2s);
        dma_status = AUDIO_I2S_DMA_STARTED;
    }
}

void audio_i2s_stop()
{
    if(dma_status != AUDIO_I2S_DMA_OFF)
    {
        HAL_I2S_DMAStop(&hi2s); 
        dma_status = AUDIO_I2S_DMA_OFF;
    }
    memset(i2s_data_buffer, 0, sizeof(i2s_data_buffer));
}

#if I2S_BUFFER_USE_TASK
static void AudioI2sTask(void const * arg)
{
    (void)arg;
    audio_info_t     *pAudioInfo;
    while(!(pAudioInfo = getAudioInfoPtr()))
    {
        osDelay(5);
    }

    audioI2sTask.Exit = 0;

    while(audioI2sTask.Exit == 0)
    {
        osEvent evt = osSignalWait(audioI2sTask.SignalMask, osWaitForever);
        if (evt.value.signals & STOP_TASK)
        {
            break;
        }

        if (evt.value.signals & AUDIO_I2S_DATA_NTF)
        {
//--copy_tx_msg((uint8_t*)"loaddata\r\n", 10);        //ok     
            audio_i2s_buffer_load_data(i2s_next_frame_data, i2s_next_frame_size);
        }
    };
    audioI2sTask.Exit = 2;
    while(audioI2sTask.Exit == 2)
	{
		osDelay(1);
	}
}

void audio_i2s_setup_tasks(void)
{
    audioI2sTask.task_stack = NULL;
    error_e ret = create_audio_i2s_task((void *)AudioI2sTask, &audioI2sTask, (uint16_t)AUDIO_I2S_TASK_STACK_SIZE_BYTES);

    if (ret != _NO_ERR)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}

void audio_i2s_terminate(void)
{
    audio_i2s_stop();
    terminate_task(&audioI2sTask);
}

int audio_i2s_task_notify(int32_t signal)
{
    if(audioI2sTask.Handle)
    {
        //Sends the Signal to the application level via OS kernel.
        //This will add a small delay of few us, but
        //this method make sense from a program structure point of view.
        if (osSignalSet(audioI2sTask.Handle, signal) == 0x80000000)
        {
            error_handler(1, _ERR_Signal_Bad);
        }
    }
    return 0;
}
#endif
#endif