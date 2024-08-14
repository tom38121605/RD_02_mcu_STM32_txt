/**
 * @file      audio_tx.c
 *
 *
 * @brief     Decawave Application level
 *             collection of data transmission bare-metal functions for a audio tx
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

/* Includes */
#include <stdint.h>
#include <stdlib.h>

#include "HAL_error.h"
#include "HAL_uwb.h"
#if !USE_USB_AUDIO_PLAYBACK
#include "usb_uart_tx.h"
#endif
#include "circular_buffers.h"
#include "cmsis_os.h"
#include "common_audio.h"
#include "critical_section.h"
#include "deca_dbg.h"
#include "deca_device_api.h"
#include "driver_app_config.h"
#include "minmax.h"
#include "rf_tuning_config.h"
#include "task_audio_tx.h"
#if USE_I2S_AUDIO_PLAYBACK
#include "audio_i2s_speaker.h"
#endif

#if UWB_TRANS_DEBUG
static uint16_t uwb_send_ok      = 0;
static uint16_t uwb_send_skip      = 0;
#if UWB_TRANS_RETRY_ENABLE
static uint16_t uwb_response_timeout = 0;
static uint16_t uwb_response_error = 0;
static uint16_t uwb_send_failure = 0;
#endif
static uint16_t fill_buffer_error  = 0;
static uint16_t fetch_buffer_error = 0;
#if !USE_USB_AUDIO_PLAYBACK
static char debug_log_buffer[128];
#endif
#endif

#if USE_FEEDBACK_AUDIO_SYNC
static i2s_audio_feedback_sync_flag i2s_pll_clock_drift_flag = 0;
static int16_t i2s_audio_feadback_count = 0;
#endif
/* Sequence index for sent message*/
uint8_t tx_msg_sn = 0;
/* Buffer to store sent frame. */
/* The frame sent in this example is a data frame encoded as per the IEEE 802.15.4-2011 standard. It is a 21-byte frame composed of the following
 * fields:
 *     - byte 0/1: frame control (0x8861 to indicate a data frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 001 - Data frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 1 - ACK frame required from recipient device on receipt of data frame
 *       - bit 6: PAN ID Compression: 1 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted frame
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 10 - Address field contains short address
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xADAD)
 *     - byte 5/6: destination address.
 *     - byte 7/8: source address.
 *     - byte 9 to rest: MAC payload.
 *     */
static uint8_t tx_buffer[FRAME_LEN_MAX_EX];

#if UWB_TRANS_RETRY_ENABLE
static uint8_t rx_buffer[RESP_FRAME_LEN];static uint8_t rx_buffer[RESP_FRAME_LEN];
static int8_t tx_attempt_count = UWB_TX_ATTEMPT_TIMES;
#endif
static int8_t tx_done = 1;

//-----------------------------------------------------------------------------
#if UWB_TRANS_DEBUG
void audio_tx_buffer_full_notify()
{
    fill_buffer_error++;
}
#endif

void audio_tx_process_tx_frame_start(const uwb_medium_access_t access)
{
    uint8_t * data_buffer = access.context.data;
    uint16_t data_size = access.context.datalength;
    data_buffer[FRAME_SN_IDX] = tx_msg_sn;
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    dwt_writetxdata(data_size + FRAME_DATA_INDEX, data_buffer, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(data_size + FRAME_DATA_INDEX + FCS_LEN, 0, 0);    /* Zero offset in TX buffer, no ranging. */

    /* Start transmission*/
#if UWB_TRANS_RETRY_ENABLE && UWB_RX_ACK_AUTO_ENABLE
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
#else
    dwt_starttx(DWT_START_TX_IMMEDIATE);
#endif
}

void audio_tx_process_tx_frame_sent(const uwb_medium_access_t access)
{
#if UWB_TRANS_RETRY_ENABLE && !UWB_RX_ACK_AUTO_ENABLE
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
#endif
}

void audio_tx_process_rx_frame(const uwb_medium_access_t access)
{
#if UWB_TRANS_RETRY_ENABLE
    /* A frame has been received, copy it to our local buffer. */
    if (access.context.datalength == RESP_FRAME_LEN)
    {
        dwt_readrxdata(rx_buffer, access.context.datalength, 0);
        /* Check if it is the expected ACK. */
        if ((rx_buffer[FRAME_FC_IDX] == RESP_FC_0) && (rx_buffer[FRAME_FC_IDX + 1] == RESP_FC_1) && (rx_buffer[FRAME_SN_IDX] == tx_buffer[FRAME_SN_IDX]))
        {
#if USB_RX_USE_DATA_FRAME_RESP && USE_FEEDBACK_AUDIO_SYNC
            i2s_pll_clock_drift_flag = rx_buffer[RESP_DATA_INDEX];
#endif
            tx_attempt_count = UWB_TX_ATTEMPT_TIMES;
            tx_msg_sn++;
            tx_done          = 1;
#if UWB_DATA_TRANS_DEBUG_TX
            audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#endif
#if UWB_TRANS_DEBUG
            uwb_send_ok++;
            
#endif
            return;
        }
    }

    /* Restart RX immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
#endif
}

void audio_tx_process_rx_error(const uwb_medium_access_t access)
{
#if UWB_TRANS_RETRY_ENABLE
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
#endif
}

void audio_tx_process_rx_timeout(const uwb_medium_access_t access)
{
#if UWB_TRANS_RETRY_ENABLE
    if (--tx_attempt_count <= 0)
    {
#if UWB_TRANS_DEBUG
        uwb_send_failure++;
#endif
        tx_attempt_count = UWB_TX_ATTEMPT_TIMES;
        tx_msg_sn++;
        tx_done = 1;
#if UWB_DATA_TRANS_DEBUG_TX
        audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#endif
    }
    else
    {
        /* Start retransmission*/
#if UWB_RX_ACK_AUTO_ENABLE
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
#else
        dwt_starttx(DWT_START_TX_IMMEDIATE);
#endif
    }
#endif
}

/* @brief   ISR level
 *          Audio application Tx callback
 *          to be called from dwt_isr() as an TX Frame Sent call-back
 * */
void audio_tx_done_cb(const dwt_cb_data_t *txd)
{
#if !UWB_TRANS_RETRY_ENABLE
    uwb_send_ok++;
    tx_msg_sn++;
    tx_done = 1;
#if UWB_DATA_TRANS_DEBUG_TX
    audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#endif
#else

    uwb_medium_access_t access;
    access.type = UWB_ACCESS_TX_FRAME_SENT;
    access.context.status = txd->status;
    access.context.status_hi = txd->status_hi;
    access.context.rx_flags = txd->rx_flags;
    access.context.datalength = txd->datalength;
    access.context.dss_stat = txd->dss_stat;

#if UWB_TX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#else
    audio_tx_process_tx_frame_sent(access);
#endif

#endif

}

#if UWB_TRANS_RETRY_ENABLE
/* @brief   ISR level
 *          Audio application TX response received callback
 *          to be called from dwt_isr() as an RX Good Frame received call-back
 * */
void audio_tx_rx_resp_cb(const dwt_cb_data_t *txd)
{
    uwb_medium_access_t access;
    access.type = UWB_ACCESS_RX_FRAME;
    access.context.status = txd->status;
    access.context.status_hi = txd->status_hi;
    access.context.rx_flags = txd->rx_flags;
    access.context.datalength = txd->datalength;
    access.context.dss_stat = txd->dss_stat;
    
#if UWB_TX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#else
    audio_tx_process_rx_frame(access);
#endif
}

/* @brief   ISR level
 *          Audio application TX response received callback
 *          to be called from dwt_isr() as an RX Timeout call-back
 * */
void audio_tx_timeout_cb(const dwt_cb_data_t *txd)
{
#if UWB_TRANS_DEBUG
    uwb_response_timeout++;
#endif

    uwb_medium_access_t access;
    access.type = UWB_ACCESS_RX_TIMEOUT;
    access.context.status = txd->status;
    access.context.status_hi = txd->status_hi;
    access.context.rx_flags = txd->rx_flags;
    access.context.datalength = txd->datalength;
    access.context.dss_stat = txd->dss_stat;

#if UWB_TX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#else
    audio_tx_process_rx_timeout(access);
#endif
}

/* @brief   ISR level
 *          Audio application TX response received callback
 *          to be called from dwt_isr() as an RX Error call-back
 * */
void audio_tx_error_cb(const dwt_cb_data_t *txd)
{
#if UWB_TRANS_DEBUG
    uwb_response_error++;
#endif

    uwb_medium_access_t access;
    access.type = UWB_ACCESS_RX_ERROR;
    access.context.status = txd->status;
    access.context.status_hi = txd->status_hi;
    access.context.rx_flags = txd->rx_flags;
    access.context.datalength = txd->datalength;
    access.context.dss_stat = txd->dss_stat;

#if UWB_TX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
#else
    audio_tx_process_rx_error(access);
#endif
}
#endif

/**
 * @description: Start UWB TX with audio sample data
 * @param {uint16_t} data_size
 * @return {*}
 */
void audio_tx_send_data(uint16_t data_size)
{
    if (tx_done == 1)
    {
        tx_done = 0;
        if (0 == audio_buffer_fetch_data(tx_buffer + FRAME_DATA_INDEX, data_size))
        {
            uwb_medium_access_t access;
            access.type = UWB_ACCESS_TX_FRAME_START;
            access.context.datalength = data_size;
            access.context.data = tx_buffer;
            /* Always defer the UWB transmission to task from USB audio ISR*/
            audio_add_uwb_medium_access(access);
            audio_tx_task_notify(AUDIO_TX_FRAME_NTF);
        }
    }
    else
    {
#if UWB_TRANS_DEBUG
        uwb_send_skip++;
#endif
    }
}

#if USE_FEEDBACK_AUDIO_SYNC
int8_t audio_tx_get_feedback_offset()
{
    int8_t offset = 0;

    if (i2s_pll_clock_drift_flag == I2S_AUDIO_UNDERRUN)
    {
        i2s_audio_feadback_count++;
    }
    else if (i2s_pll_clock_drift_flag == I2S_AUDIO_OVERRUN)
    {
        i2s_audio_feadback_count--;
    }
    else
    {
        i2s_audio_feadback_count = 0;
    }

    if (i2s_audio_feadback_count > AUDIO_FEEDBACK_SYNC_OFFSET_STEP || i2s_audio_feadback_count < -AUDIO_FEEDBACK_SYNC_OFFSET_STEP)
    {
        offset = (i2s_audio_feadback_count / AUDIO_FEEDBACK_SYNC_OFFSET_STEP);
        i2s_audio_feadback_count = 0;
    }

    return offset;
}
#endif

void audio_tx_debug_output()
{
#if UWB_TRANS_DEBUG
    if (uwb_send_ok || fill_buffer_error || fetch_buffer_error || uwb_send_skip
#if UWB_TRANS_RETRY_ENABLE
    || uwb_response_error || uwb_response_timeout || uwb_send_failure
#endif
    )
    {
#if !USE_USB_AUDIO_PLAYBACK
#if UWB_TRANS_RETRY_ENABLE
        sprintf(debug_log_buffer, "\"S:%5d\",\"E:%5d\",\"T:%5d\",\"F:%5d\",\"Buffer:%5d-%5d\",\"Skip:%5d\"\r\n", uwb_send_ok, 
        uwb_response_error, uwb_response_timeout, uwb_send_failure, fill_buffer_error, fetch_buffer_error, uwb_send_skip);
#else
        sprintf(debug_log_buffer, "\"S:%5d\",\"Buffer:%5d-%5d\",\"Skip:%5d\"\r\n", uwb_send_ok, fill_buffer_error, fetch_buffer_error, uwb_send_skip);
#endif
        copy_tx_msg((uint8_t*)debug_log_buffer, strlen(debug_log_buffer));
#endif
        // STM32CubeIDE Breakpoint log "%dR-Buffer:%d-%d\r\n", uwb_send_ok, fill_buffer_error, fetch_buffer_error
        // VSCode Breakpoint log:  "%dR%dT%dE-Buffer:%d-%d\r\n" uwb_send_ok uwb_response_timeout uwb_response_error fill_buffer_error fetch_buffer_error
        uwb_send_ok   = 0;
#if UWB_TRANS_RETRY_ENABLE
        uwb_response_timeout = 0;
        uwb_response_error   = 0;
        uwb_send_failure = 0;
#endif
        fill_buffer_error  = 0;
        fetch_buffer_error = 0;
        uwb_send_skip = 0;
    }
#endif
}

/* @brief     app level
 *     RTOS-independent application level function.
 *     initializing of a Audio TX functionality.
 *
 * */
error_e audio_tx_process_init()
{
    /* Configure non-zero initial variables.1 : from app parameters */

    /* The AudioTx has its configuration in the app->pConfig, see DEFAULT_CONFIG.
     *
     *
     * */
    dwt_config_t *dwt_config = get_dwt_config();

    /* dwt_xx calls in app level Must be in protected mode (DW3000 IRQ disabled) */
    hal_uwb.disableIRQ();

    AUDIO_ENTER_CRITICAL();

    if (dwt_initialise(0) != DWT_SUCCESS) /**< set callbacks to NULL inside dwt_initialise*/
    {
        return (_ERR_INIT);
    }

    if (hal_uwb.uwbs != NULL)
    {
        hal_uwb.uwbs->spi->fast_rate(hal_uwb.uwbs->spi->handler);
    }

    /* Configure DW IC's UWB mode, sets power and antenna delays for Audio Tx mode
     * Configure SPI to fast rate */
    rf_tuning_t *rf_tuning = get_rf_tuning_config();
    audio_configure_uwb_rxtx(dwt_config,
#if USB_RX_USE_DATA_FRAME_RESP
                             DWT_FF_DATA_EN, /* Allow only for DATA FRAME */
#else
                             DWT_FF_ACK_EN, /* Allow only for ACT FRAME*/
#endif
                             rf_tuning->antTx_a,
                             rf_tuning->antRx_a);
#if UWB_TRANS_RETRY_ENABLE
    audio_uwb_init(audio_tx_done_cb, audio_tx_rx_resp_cb, audio_tx_timeout_cb, audio_tx_error_cb, rf_tuning->xtalTrim);
#else
    audio_uwb_init(audio_tx_done_cb, NULL, NULL, NULL, rf_tuning->xtalTrim);
#endif

    /* End configuration of DW IC */

    AUDIO_EXIT_CRITICAL();

    memset(tx_buffer, 0x0, sizeof(tx_buffer));
    tx_buffer[0] = FRAME_FC_0;
    tx_buffer[1] = FRAME_FC_1;
    tx_buffer[2] = 0x0;
    tx_buffer[3] = PAN_ID_LO;
    tx_buffer[4] = PAN_ID_HI;
    tx_buffer[5] = DEST_SHORT_ADDR_LO;
    tx_buffer[6] = DEST_SHORT_ADDR_HI;
    tx_buffer[7] = SHORT_ADDR_LO;
    tx_buffer[8] = SHORT_ADDR_HI;

    audio_buffer_init();
    hal_uwb.enableIRQ();

    return (_NO_ERR);
}


/* @brief     app level
 *     RTOS-independent application level function.
 *     deinitialize the pAudioInfo structure.
 *    This must be executed in protected mode.
 *
 * */
void audio_tx_process_deinit(void)
{
    tx_done = 1;
    audio_uwb_deinit();
    audio_buffer_deinit();
}

//-----------------------------------------------------------------------------
