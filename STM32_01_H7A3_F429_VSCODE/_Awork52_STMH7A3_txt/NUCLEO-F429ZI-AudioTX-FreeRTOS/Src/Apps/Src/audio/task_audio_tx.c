/**
 * @file      task_audio_tx.c
 *
 * @brief     AudioTx task functionalities.
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

#include <math.h>
#include "cmsis_os.h"
#include "critical_section.h"
#include "deca_dbg.h"
#include "app.h"
#include "HAL_error.h"
#include "HAL_uwb.h"
#include "cmd_fn.h"
#include "cmd.h"
#include "create_audio_tx_task.h"
#include "driver_app_config.h"
#include "rf_tuning_config.h"
#include "common_audio.h"
#include "audio_tx.h"
#include "task_audio_tx.h"
#if UWB_DATA_TRANS_DEBUG_TX
#include "HAL_watchdog.h"
#endif
#if I2S_BUFFER_USE_TASK
#include "task_audio_i2s.h"
#endif

static task_signal_t audioTxTask;
#if UWB_DATA_TRANS_DEBUG_TX
static uint8_t tx_buffer[AUDIO_UWB_PACKAGE_SIZE]={0};
#endif

extern const struct command_s known_commands_audio_tx;

#define AUDIO_TX_TASK_STACK_SIZE_BYTES 2048

//-----------------------------------------------------------------------------
/* @brief DW3000 RX : AudioTx RTOS implementation
 *          this is a high-priority task, which will be executed immediately
 *          on reception of waiting Signal. Any task with lower priority will be interrupted.
 *          No other tasks in the system should have higher priority.
 * */
static void AudioTxTask(void const * arg)
{
    (void)arg;
    audio_info_t     *pAudioTxInfo;
    while(!(pAudioTxInfo = getAudioInfoPtr()))
    {
        osDelay(5);
    }

    audioTxTask.Exit = 0;
    #if UWB_DATA_TRANS_DEBUG_TX
    memset(tx_buffer, 0x0, sizeof(tx_buffer));
    audio_tx_task_fill_data(tx_buffer, AUDIO_UWB_PACKAGE_SIZE);
    #endif

    while(audioTxTask.Exit == 0)
    {
        osEvent evt = osSignalWait(audioTxTask.SignalMask, osWaitForever);
        if (evt.value.signals & STOP_TASK)
        {
            break;
        }

        if (evt.value.signals & AUDIO_TX_FRAME_NTF)
        {
            uwb_medium_access_t access;
            if (audio_get_uwb_medium_access(&access) == 0)
            {
                switch (access.type)
                {
                case UWB_ACCESS_TX_FRAME_START:
                    audio_tx_process_tx_frame_start(access);
                    break;
                case UWB_ACCESS_TX_FRAME_SENT:
                    audio_tx_process_tx_frame_sent(access);
                    break;
                case UWB_ACCESS_RX_FRAME:
                    audio_tx_process_rx_frame(access);
                    break;
                case UWB_ACCESS_RX_ERROR:
                    audio_tx_process_rx_error(access);
                    break;
                case UWB_ACCESS_RX_TIMEOUT:
                    audio_tx_process_rx_timeout(access);
                    break;
                default:
                    break;
                }
            }
#if UWB_DATA_TRANS_DEBUG_TX
            else
            {
                osDelay(1);
                audio_tx_task_fill_data(tx_buffer, AUDIO_UWB_PACKAGE_SIZE);
            }
#endif
        }
    };
    audioTxTask.Exit = 2;
    while(audioTxTask.Exit == 2)
	{
		osDelay(1);
	}
}

void audio_tx_update_package_size(uint16_t ms_audio_sample_size)
{
    audio_update_package_size(ms_audio_sample_size);
}

void audio_tx_buffer_reset()
{
    audio_buffer_reset();
}

/* @brief Setup AudioTx task, this task will send out buffered audio data.
 * Only setup, do not start.
 * */
static void audio_tx_setup_tasks(void)
{
    /* audio_txTask is receiving the audio data from USB interrupt and passing to UWB chip for transmission. */
    audioTxTask.task_stack = NULL;
    error_e ret = create_audio_tx_task((void *)AudioTxTask, &audioTxTask, (uint16_t)AUDIO_TX_TASK_STACK_SIZE_BYTES);

    if (ret != _NO_ERR)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}

/* @brief Terminate all tasks and timers related to audio TX functionality, if any
 *        DW3000's TX and IRQ shall be switched off before task termination,
 *        that IRQ will not produce unexpected Signal
 * */
void audio_tx_terminate(void)
{
    /*   need to switch off DW chip's TX before killing tasks */
    hal_uwb.disableIRQ();
    hal_uwb.reset();

    terminate_task(&audioTxTask);
#if USE_I2S_AUDIO_PLAYBACK && I2S_BUFFER_USE_TASK
    audio_i2s_terminate();
#endif

    audio_tx_process_deinit();
}

/* @fn         audio_tx_helper
 * @brief      this is a service function which starts the
 *             audio tx functionality
 *
 * */
void audio_tx_helper(void const *argument)
{
    error_e   tmp;

    hal_uwb.disable_irq_and_reset(1);

    /* "RTOS-independent" part : initialization of two-way ranging process */
    tmp = audio_tx_process_init();    /* allocate AudioRxInfo */

    if(tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    audio_tx_setup_tasks();
#if USE_I2S_AUDIO_PLAYBACK && I2S_BUFFER_USE_TASK
    audio_i2s_setup_tasks();
#endif
}

/**
 * @description: Fill audio sample to a circle buffer which will send out later by UWB TX task
 * @param {uint8_t} *data_buffer
 * @param {uint16_t} data_size
 * @return {*}
 */
int audio_tx_task_fill_data(uint8_t *data_buffer, uint16_t data_size)
{
    if(audioTxTask.Handle)         // RTOS : listenerTask can be not started yet
    {
        if (0 != audio_buffer_fill_data(data_buffer, data_size))
        {
#if UWB_TRANS_DEBUG
            audio_tx_buffer_full_notify();
#endif
            return -1;
        }
        
        uint16_t package_size = getAudioInfoPtr()->package_size;
        if (0 == audio_buffer_read_check(package_size))
        {
            audio_tx_send_data(package_size);
        } 
    }
    return 0;
}

int audio_tx_task_notify(int32_t signal)
{
    if(audioTxTask.Handle)
    {
        //Sends the Signal to the application level via OS kernel.
        //This will add a small delay of few us, but
        //this method make sense from a program structure point of view.
        if (osSignalSet(audioTxTask.Handle, signal) == 0x80000000)
        {
            error_handler(1, _ERR_Signal_Bad);
        }
    }
    return 0;
}

bool audio_tx_task_started(void)
{
    return audioTxTask.Handle != NULL;
}

const app_definition_t helpers_app_audio_tx[] __attribute__((section(".known_apps"))) = 
{
    {"AUDIO_TX", mAPP, audio_tx_helper,  audio_tx_terminate, waitForCommand, command_parser, &known_commands_audio_tx}
};
