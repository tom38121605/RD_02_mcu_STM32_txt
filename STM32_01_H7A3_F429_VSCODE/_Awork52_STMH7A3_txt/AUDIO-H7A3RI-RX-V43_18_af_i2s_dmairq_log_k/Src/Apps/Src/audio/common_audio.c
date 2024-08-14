/**
 *  @file     common_audio.c
 *
 *  @brief    Audio data transmit buffer control
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
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>

#include "HAL_error.h"
#include "common_audio.h"
#include "deca_dbg.h"
#include "deca_device_api.h"
#include "driver_app_config.h"

#include "usb_uart_tx.h"  //--add


//-----------------------------------------------------------------------------
// The psAudioInfo structure holds all Audio's process parameters
static audio_info_t audioInfo = {
    NULL,
    0,
    0,
};
static audio_data_circular_buffer_t audio_data_circular_buffer;
static audio_info_t *psAudioInfo = NULL;
static uint8_t audio_data_buffer[AUDIO_UWB_BUFFER_SIZE];

static uwb_medium_access_t uwb_medium_access_buffer[UWB_MEDIUM_ACCESS_BUFFER_SIZE];

/*
 * @brief     get pointer to the audio info structure
 * */
audio_info_t *getAudioInfoPtr(void)
{
    return (psAudioInfo);
}

/**
 * @description: Helper function to check and wait specific DW IC system status register been set
 * @param {uint32_t} *lo_result
 * @param {uint32_t} *hi_result
 * @param {uint32_t} lo_mask
 * @param {uint32_t} hi_mask
 * @return {*}
 */
void audio_wait_for_sys_status(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask)
{
    uint32_t lo_result_tmp = 0;
    uint32_t hi_result_tmp = 0;

    // If a mask has been passed into the function for the system status register (lower 32-bits)
    if (lo_mask)
    {
        while (!((lo_result_tmp = dwt_readsysstatuslo()) & (lo_mask)))
        {
            // If a mask value is set for the system status register (higher 32-bits)
            if (hi_mask)
            {
                // If mask value for the system status register (higher 32-bits) is found
                if ((hi_result_tmp = dwt_readsysstatushi()) & hi_mask)
                {
                    break;
                }
            }
        }
    }
    // if only a mask value for the system status register (higher 32-bits) is set
    else if (hi_mask)
    {
        while (!((hi_result_tmp = dwt_readsysstatushi()) & (hi_mask)))
        {
        };
    }

    if (lo_result != NULL)
    {
        *lo_result = lo_result_tmp;
    }

    if (hi_result != NULL)
    {
        *hi_result = hi_result_tmp;
    }
}

/**
 * @description: Update UWB data transmission package size
 * @param {uint16_t} ms_audio_sample_size
 * @return {*}
 */
void audio_update_package_size(uint16_t ms_audio_sample_size)
{
    psAudioInfo->ms_audio_sample_size = ms_audio_sample_size;
    audio_buffer_reset();
}

/**
 * @description: Init UWB transmission buffer for audio
 * @return {*}
 */
int audio_buffer_init()
{
    if (!psAudioInfo)
    {
        psAudioInfo = &audioInfo;
    }

    memset(psAudioInfo, 0, sizeof(audio_info_t));

    psAudioInfo->ms_audio_sample_size = AUDIO_MS_SAMPLE_SIZE;
    psAudioInfo->package_size         = AUDIO_UWB_PACKAGE_SIZE;

    psAudioInfo->data_buffer = &audio_data_circular_buffer;
    memset(psAudioInfo->data_buffer, 0x0, sizeof(audio_data_circular_buffer_t));
    psAudioInfo->data_buffer->size   = AUDIO_UWB_BUFFER_SIZE;
    psAudioInfo->data_buffer->data   = audio_data_buffer;
    psAudioInfo->data_buffer->wr_ptr = 0;
    psAudioInfo->data_buffer->rd_ptr = 0;
    if (!psAudioInfo->data_buffer->data)
    {
        return -1;
    }
    memset(psAudioInfo->data_buffer->data, 0xA, psAudioInfo->data_buffer->size);

    psAudioInfo->access_buffer = uwb_medium_access_buffer;
    psAudioInfo->next_access_index = 0;
    psAudioInfo->last_access_index = 0;
    return (_NO_ERR);
}

/**
 * @description: DeInit UWB transmission buffer for audio
 * @return {*}
 */
int audio_buffer_deinit()
{
    if (psAudioInfo)
    {
        audio_buffer_reset();
        psAudioInfo->data_buffer->data = NULL;
        psAudioInfo->data_buffer = NULL;
        psAudioInfo = NULL;
    }
    return (_NO_ERR);
}

/**
 * @description: Check if there is enough space for write in audio buffer
 * @param {uint16_t} data_size required size
 * @return {*}
 */
int audio_buffer_write_check(uint16_t data_size)
{
    uint16_t buffer_data_size = audio_buffer_data_size();

    if (psAudioInfo->data_buffer->size - buffer_data_size - 1 < data_size)
        return -1;
    return 0;
}

/**
 * @description: Check if there is enough data for read in audio buffer
 * @param {uint16_t} data_size required size
 * @return {*}
 */
int audio_buffer_read_check(uint16_t data_size)
{
    uint16_t buffer_data_size = audio_buffer_data_size();

    if (buffer_data_size < data_size)
        return -1;
    return 0;
}

/**
 * @description: Update audio buffer read pointer
 * @param {uint16_t} new_rd_ptr
 * @return {*}
 */
void audio_buffer_update_rd_ptr(uint16_t new_rd_ptr)
{
    psAudioInfo->data_buffer->rd_ptr = new_rd_ptr;
}

/**
 * @description: Update audio buffer write pointer
 * @param {uint16_t} new_wr_ptr
 * @return {*}
 */
void audio_buffer_update_wr_ptr(uint16_t new_wr_ptr)
{
    psAudioInfo->data_buffer->wr_ptr = new_wr_ptr;
}

/**
 * @description: Get audio buffer filled data size
 * @return {*}
 */
int audio_buffer_data_size()
{
    return (psAudioInfo->data_buffer->wr_ptr + psAudioInfo->data_buffer->size - psAudioInfo->data_buffer->rd_ptr) % psAudioInfo->data_buffer->size;
}

/**
 * @description: Reset audio buffer
 * @return {*}
 */
void audio_buffer_reset()
{
    /* Clearing rx/tx buffer content */
    psAudioInfo->data_buffer->wr_ptr = 0;
    psAudioInfo->data_buffer->rd_ptr = 0;
    memset(psAudioInfo->data_buffer->data, 0x0, psAudioInfo->data_buffer->size);

    /* Clearing uwb medium access buffer index */
    psAudioInfo->next_access_index = 0;
    psAudioInfo->last_access_index = 0;
}

/**
 * @description: Fill audio data into UWB transmission buffer
 * @param {uint8_t} *data_buffer
 * @param {uint16_t} data_size
 * @return {*}
 */
int audio_buffer_fill_data(uint8_t *data_buffer, uint16_t data_size)
{
    if (0 == audio_buffer_write_check(data_size))
    {
        if (psAudioInfo->data_buffer->wr_ptr + data_size > psAudioInfo->data_buffer->size)
        {
copy_tx_msg((uint8_t*)"f11\r\n", 5);  //no
            uint16_t tail_size = psAudioInfo->data_buffer->size - psAudioInfo->data_buffer->wr_ptr;
            uint16_t front_size = data_size - tail_size;
            memcpy(psAudioInfo->data_buffer->data + psAudioInfo->data_buffer->wr_ptr, data_buffer, tail_size);
            memcpy(psAudioInfo->data_buffer->data, data_buffer+tail_size, front_size);
        }
        else
        {
//--copy_tx_msg((uint8_t*)"f12\r\n", 5);  //ok
            memcpy(psAudioInfo->data_buffer->data + psAudioInfo->data_buffer->wr_ptr, data_buffer, data_size);
        }
        psAudioInfo->data_buffer->wr_ptr = (psAudioInfo->data_buffer->wr_ptr + data_size) % psAudioInfo->data_buffer->size;
        return 0;
    }
    return -1;
}

/**
 * @description: Get audio data from UWB tranmission buffer
 * @param {uint8_t} *data_buffer
 * @param {uint16_t} data_size
 * @return {*}
 */
int audio_buffer_fetch_data(uint8_t *data_buffer, uint16_t data_size)
{
    if (0 == audio_buffer_read_check(data_size))
    {
        if (psAudioInfo->data_buffer->rd_ptr + data_size > psAudioInfo->data_buffer->size)
        {
            uint16_t tail_size = psAudioInfo->data_buffer->size - psAudioInfo->data_buffer->rd_ptr;
            uint16_t front_size = data_size - tail_size;
            memcpy(data_buffer, psAudioInfo->data_buffer->data + psAudioInfo->data_buffer->rd_ptr, tail_size);
            memcpy(data_buffer+tail_size, psAudioInfo->data_buffer->data, front_size);
        }
        else
        {
            memcpy(data_buffer, psAudioInfo->data_buffer->data + psAudioInfo->data_buffer->rd_ptr, data_size);
        }
        psAudioInfo->data_buffer->rd_ptr = (psAudioInfo->data_buffer->rd_ptr + data_size) % psAudioInfo->data_buffer->size;
        return 0;
    }
    return -1;
}

int audio_add_uwb_medium_access(const uwb_medium_access_t access)
{
    uint16_t current_last = psAudioInfo->last_access_index;
    uint16_t current_next = psAudioInfo->next_access_index;
    uint16_t new_last = (current_last + 1 + UWB_MEDIUM_ACCESS_BUFFER_SIZE) % UWB_MEDIUM_ACCESS_BUFFER_SIZE;
    if (new_last != current_next)
    {
        uwb_medium_access_t * new_access = &(psAudioInfo->access_buffer[current_last]);
        new_access->type = access.type;
        new_access->context.status = access.context.status;
        new_access->context.status_hi = access.context.status_hi;
        new_access->context.rx_flags = access.context.rx_flags;
        new_access->context.data = access.context.data;
        new_access->context.datalength = access.context.datalength;
        new_access->context.dss_stat = access.context.dss_stat;

        psAudioInfo->last_access_index = new_last;
        return 0;
    }
    return -1;
}

int audio_get_uwb_medium_access(uwb_medium_access_t * access)
{
    uint16_t current_last = psAudioInfo->last_access_index;
    uint16_t current_next = psAudioInfo->next_access_index;
    
    if (current_last != current_next)
    {
        uint16_t new_next = (current_next + 1 + UWB_MEDIUM_ACCESS_BUFFER_SIZE) % UWB_MEDIUM_ACCESS_BUFFER_SIZE;
        *access = psAudioInfo->access_buffer[current_next];
        psAudioInfo->next_access_index = new_next;
        return 0;
    }
    return -1;
}