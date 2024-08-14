/**
 * @file      task_audio_rx.c
 *
 * @brief     AudioRx task functionalities
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
#include "HAL_error.h"
#include "HAL_uwb.h"
#include "app.h"
#include "cmd.h"
#include "cmd_fn.h"
#include "cmsis_os.h"
#include "critical_section.h"
#include "common_audio.h"
#include "create_audio_rx_task.h"
#include "task_audio_rx.h"
#include "audio_rx.h"
#if I2S_BUFFER_USE_TASK
#include "task_audio_i2s.h"
#endif

extern const struct command_s known_commands_audio_rx;

#if UWB_RX_USE_TASK
static task_signal_t audioRxTask;

#define AUDIO_RX_TASK_STACK_SIZE_BYTES 2048

//-----------------------------------------------------------------------------
/* @brief DW3000 RX : AudioRx RTOS implementation
 *          this is a high-priority task, which will be executed immediately
 *          on reception of waiting Signal. Any task with lower priority will be interrupted.
 *          No other tasks in the system should have higher priority.
 * */
static void AudioRxTask(void const * arg)
{
    (void)arg;
    audio_info_t     *pAudioRxInfo;
    while(!(pAudioRxInfo = getAudioInfoPtr()))
    {
        osDelay(5);
    }

    audioRxTask.Exit = 0;

    while(audioRxTask.Exit == 0)
    {
        osEvent evt = osSignalWait(audioRxTask.SignalMask, osWaitForever);
        if (evt.value.signals & STOP_TASK)
        {
            break;
        }

        if (evt.value.signals & AUDIO_RX_FRAME_NTF)
        {
            uwb_medium_access_t access;
            if (audio_get_uwb_medium_access(&access) == 0)
            {
                switch (access.type)
                {
                case UWB_ACCESS_RX_FRAME:
                    audio_rx_process_rx_frame(access);
                    break;
                case UWB_ACCESS_TX_FRAME_SENT:
                    audio_rx_process_tx_frame_sent(access);
                    break;
                case UWB_ACCESS_RX_ERROR:
                    audio_rx_process_rx_error(access);
                    break;
                case UWB_ACCESS_RX_TIMEOUT:
                    audio_rx_process_rx_timeout(access);
                    break;
                default:
                    break;
                }
            }
        }
    };
    audioRxTask.Exit = 2;
    while(audioRxTask.Exit == 2)
	{
		osDelay(1);
	}
}

int audio_rx_task_notify(int32_t signal)
{
    if(audioRxTask.Handle)         // RTOS : listenerTask can be not started yet
    {
        //Sends the Signal to the application level via OS kernel.
        //This will add a small delay of few us, but
        //this method make sense from a program structure point of view.
        if (osSignalSet(audioRxTask.Handle, signal) == 0x80000000)
        {
            error_handler(1, _ERR_Signal_Bad);
        }
    }
    return 0;
}

bool audio_rx_task_started(void)
{
    return audioRxTask.Handle != NULL;
}

/* @brief Setup AudioRx task, this task will process every received UWB package.
 * Only setup, do not start.
 * */
static void audio_rx_setup_tasks(void)
{
    /* audio_rxTask is receiving the UWB package from RX IRQ to audio circle buffer.
     * It awaiting of an Rx Signal from RX IRQ ISR and decides what to do next in Audio Rx exchange process
     * */
    audioRxTask.task_stack = NULL;
    error_e ret = create_audio_rx_task((void *)AudioRxTask, &audioRxTask, (uint16_t)AUDIO_RX_TASK_STACK_SIZE_BYTES);

    if (ret != _NO_ERR)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}
#endif

/* @brief Terminate all tasks and timers related to Node functionality, if any
 *        DW3000's RX and IRQ shall be switched off before task termination,
 *        that IRQ will not produce unexpected Signal
 * */
void audio_rx_terminate(void)
{
    /*   need to switch off DW chip's RX and IRQ before killing tasks */
    hal_uwb.disableIRQ();
    hal_uwb.reset();

#if UWB_RX_USE_TASK
    terminate_task(&audioRxTask);
#endif
#if USE_I2S_AUDIO_PLAYBACK && I2S_BUFFER_USE_TASK
    audio_i2s_terminate();
#endif
    audio_rx_process_deinit();
    hal_uwb.sleep_enter();
}

/* @fn         audio_rx_helper
 * @brief      this is a service function which starts the
 *             Audio Rx functionality
 *             Note: the previous instance of Audio Rx shall be killed
 *             with node_terminate_tasks();
 *
 *             Note: the node_process_init() will allocate the memory of sizeof(node_info_t)
 *                   from the <b>caller's</b> task stack, see _malloc_r() !
 *
 * */
void audio_rx_helper(void const *argument)
{
    error_e tmp;

    hal_uwb.disable_irq_and_reset(1);

    /* "RTOS-independent" part : initialization of audio receiving process */
    tmp = audio_rx_process_init();

    if (tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }
#if UWB_RX_USE_TASK
    audio_rx_setup_tasks();
#endif
#if USE_I2S_AUDIO_PLAYBACK && I2S_BUFFER_USE_TASK
    audio_i2s_setup_tasks();
#endif
    /**< IRQ is enabled from MASTER chip and it may receive UWB immediately after this point */
    audio_rx_process_start();
}

const app_definition_t helpers_app_audio_rx[] __attribute__((section(".known_apps"))) = {
    {"AUDIO_RX", mAPP, audio_rx_helper, audio_rx_terminate, waitForCommand, command_parser, &known_commands_audio_rx}};