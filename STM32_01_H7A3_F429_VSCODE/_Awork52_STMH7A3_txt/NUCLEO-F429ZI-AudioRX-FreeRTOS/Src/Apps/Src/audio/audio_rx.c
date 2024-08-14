/**
 * @file      audio_rx.c
 *
 *
 * @brief     Decawave Application level
 *             collection of data transmission bare-metal functions for audio RX
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
#include <stdlib.h>
#include <stdint.h>
// #include "stm32f4xx_hal.h"
#include "HAL_error.h"
#include "HAL_uwb.h"
#include "common_audio.h"
#include "audio_rx.h"
#include "circular_buffers.h"
#include "cmsis_os.h"
#include "critical_section.h"
#include "deca_dbg.h"
#include "deca_device_api.h"
#include "driver_app_config.h"
#include "minmax.h"
#include "rf_tuning_config.h"
#include "task_audio_rx.h"
/* Enable USB-UART log output if USB port haven't been used as audio IO*/
#if !USE_USB_AUDIO_PLAYBACK && !USE_USB_AUDIO_RECORDING
#include "usb_uart_tx.h"
#endif

/* Local definitoins for audio output interface USB or I2S */
#if USE_I2S_AUDIO_PLAYBACK
#include "audio_i2s_speaker.h"
static uint8_t i2s_alt_buffer[AUDIO_I2S_DATA_SIZE] = {0};
static uint16_t i2s_pll_num = I2S_PLL_DEFAULT_NUM_N;
#if USE_PLL_CLOCK_AUDIO_SYNC
#include "stm32f4xx_hal_rcc_ex.h"
#else
static i2s_audio_feedback_sync_flag i2s_pll_clock_drift_flag = 0;
#endif
#elif USE_USB_AUDIO_RECORDING
#define TIMER_SYNC_DEFAULT_PERIOD 999
#define TIMER_SYNC_STEP 1
static uint16_t timer_period= TIMER_SYNC_DEFAULT_PERIOD;

extern TIM_HandleTypeDef htim4;
static uint8_t timer_data_buffer[AUDIO_MS_SAMPLE_SIZE] = {0};
#endif

#if UWB_TRANS_DEBUG
/* Counters for debug */
static uint16_t uwb_receive_ok            = 0;
static uint16_t uwb_receive_error              = 0;
static uint16_t uwb_recieve_timeout            = 0;
static uint16_t uwb_receive_msg_error       = 0;
static uint16_t uwb_receive_msg_retry         = 0;
static uint16_t fill_buffer_error             = 0;
static uint16_t fetch_buffer_error            = 0;
static uint16_t audio_data_output_error = 0;
static uint32_t audio_data_output_Call       = 0;
static uint16_t audio_i2s_pll_updated = 0;
#if !USE_USB_AUDIO_RECORDING
static char debug_log_buffer[128];
#endif
#endif

#define RX_BUFFER_DELAY_COUNT 5
/* Buffer to store received frame. */
static uint8_t rx_buffer[FRAME_LEN_MAX_EX];
/* Sequence index for received message*/
static uint8_t rx_msg_sn = 0;
/* Audio streaming status */
static uint8_t rx_streaming_started = FALSE;

#if UWB_TRANS_RETRY_ENABLE && !UWB_RX_ACK_AUTO_ENABLE
/* Buffer to store RESP frame */
/* The frame sent in this example is a data frame encoded as per the IEEE 802.15.4-2011 standard. It is a 21-byte frame composed of the following
 * fields:
 *     - byte 0/1: frame control (0x8861 to indicate a data frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 001 - Data frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 0 - ACK frame not required from recipient device on receipt of data frame
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
static uint8_t rx_resp_buffer[RESP_FRAME_LEN];
#endif

/* Call back function to transfer received audio data to USB port*/
#if USE_USB_AUDIO_RECORDING
extern void UWB_AUDIO_IN_TransferComplete_CallBack(uint8_t *data_buffer, uint16_t data_size);
#endif

// ----------------------------------------------------------------------------
/**
 * @description: Handle received UWB package 
 * @return {*}
 */
void audio_rx_data_in()
{
    uint8_t * rx_audio_data = rx_buffer + FRAME_DATA_INDEX;
    audio_info_t * info = getAudioInfoPtr();
    uint16_t data_size = info->package_size;
    uint8_t new_msg_sn = rx_buffer[FRAME_SN_IDX];

    /* Check received package's frame control sequence and store audio data accordingly*/
    ++rx_msg_sn;
    if (rx_streaming_started)
    {
        /* Expected sequence, save to audio buffer*/
        if (rx_msg_sn == new_msg_sn)
        {
            audio_buffer_fill_data(rx_audio_data, data_size);
        }
#if UWB_TRANS_RETRY_ENABLE && UWB_TRANS_DEBUG
        /* Duplicate sequence due to TX retransmission, mark and ignore. This happen when transceiver miss ACK from receiver and start retransmission. */
        else if (--rx_msg_sn == new_msg_sn)
        {
            uwb_receive_msg_retry++;
        }
#endif
        /* Unexpected sequence, packages lost during transmission. */
        else
        {
            audio_buffer_fill_data(rx_audio_data, data_size);
#if UWB_TRANS_DEBUG
            uwb_receive_msg_error++;
            sprintf(debug_log_buffer, "T:%dR:%d\r\n", rx_msg_sn+1, new_msg_sn);
            copy_tx_msg((uint8_t*)debug_log_buffer, strlen(debug_log_buffer));
#endif
        }
    }
    else
    {
#if UWB_TRANS_DEBUG
        if (audio_buffer_data_size() != 0 && rx_msg_sn != new_msg_sn)
        {
#if UWB_TRANS_RETRY_ENABLE
            if (--rx_msg_sn == new_msg_sn)
            {
                uwb_receive_msg_retry++;
            }
            else
#endif
            {
                uwb_receive_msg_error++;
                sprintf(debug_log_buffer, "T:%dR:%d\r\n", rx_msg_sn+1, new_msg_sn);
                copy_tx_msg((uint8_t*)debug_log_buffer, strlen(debug_log_buffer));
            }
        }
#endif
        
        /* Audio streaming not start yet, fill audio buffer*/
        audio_buffer_fill_data(rx_audio_data, data_size);
    }

    /* Always update the frame control sequence to latest package*/
    rx_msg_sn = new_msg_sn;

    /* Start audio streaming when received data exceed half buffer size*/
    if (audio_buffer_data_size() >= info->data_buffer->size / 2)
    {
        if (!rx_streaming_started)
        {
            rx_streaming_started = TRUE;
            audio_rx_process_trigger();
        }
#if UWB_DATA_TRANS_DEBUG_RX
        // Ignore filled data for debug
        info->data_buffer->rd_ptr = (info->data_buffer->rd_ptr + info->package_size) % info->data_buffer->size;
#endif
    }
}

void audio_rx_process_rx_frame(const uwb_medium_access_t access)
{
    /* ready to serve next raw reception */
#if !UWB_TRANS_RETRY_ENABLE
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
#endif

    uint16_t size = getAudioInfoPtr()->package_size + FCS_LEN + FRAME_DATA_INDEX;
    /* A frame has been received, copy it to our local buffer. */
    if (access.context.datalength == size)
    {
#if UWB_TRANS_DEBUG
        uwb_receive_ok++;
#endif
        uint16_t rx_buffer_offset = 0;
#if UWB_TRANS_RETRY_ENABLE && !UWB_RX_ACK_AUTO_ENABLE
#if USB_RX_USE_DATA_FRAME_RESP && USE_I2S_AUDIO_PLAYBACK && USE_FEEDBACK_AUDIO_SYNC
        rx_resp_buffer[RESP_DATA_INDEX] = i2s_pll_clock_drift_flag;
#endif
        /* Read out frame sn and send response. RX Double buffer must be DISABLED*/
        dwt_readrxdata(rx_buffer, FRAME_DATA_INDEX, rx_buffer_offset);
        rx_buffer_offset += FRAME_DATA_INDEX;
        size -= FRAME_DATA_INDEX;
        rx_resp_buffer[RESP_SN_IDX] = rx_buffer[FRAME_SN_IDX];
        
        /* Write response frame data with received frame sn*/
        dwt_writetxdata(sizeof(rx_resp_buffer)-FCS_LEN, rx_resp_buffer, 0);
        dwt_writetxfctrl(sizeof(rx_resp_buffer), 0, 0);      
        dwt_starttx(DWT_START_TX_IMMEDIATE);
#endif
        /*If using task to handle receiving data instead of ISR, disable interrupt to avoid system hangup.
        Every DWT driver call will lock SPI bus for transmission. If a UWB interrupt trigger and acquire
        SPI bus, which have already been locked by dwt_readrxdata, in ISR, system will been hang due to this
        race condition.*/
#if UWB_RX_USE_TASK
        hal_uwb.disableIRQ();
        dwt_readrxdata(rx_buffer+rx_buffer_offset, size, rx_buffer_offset);
        hal_uwb.enableIRQ();
#else
        dwt_readrxdata(rx_buffer+rx_buffer_offset, size, rx_buffer_offset);
#endif

        audio_rx_data_in();
    }
}

void audio_rx_process_tx_frame_sent(const uwb_medium_access_t access)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void audio_rx_process_rx_error(const uwb_medium_access_t access)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void audio_rx_process_rx_timeout(const uwb_medium_access_t access)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* @brief   ISR level
 *          Audio application Rx callback
 *          to be called from dwt_isr() as an Rx call-back
 * */
void audio_rx_done_cb(const dwt_cb_data_t *rxd)
{
    uwb_medium_access_t access;
    access.type = UWB_ACCESS_RX_FRAME;
    access.context.status = rxd->status;
    access.context.status_hi = rxd->status_hi;
    access.context.rx_flags = rxd->rx_flags;
    access.context.datalength = rxd->datalength;
    access.context.dss_stat = rxd->dss_stat;
    
#if UWB_RX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_rx_task_notify(AUDIO_RX_FRAME_NTF);
#else
    audio_rx_process_rx_frame(access);
#endif
}

#if UWB_TRANS_RETRY_ENABLE
/* @brief   ISR level
 *          Audio application RX response tx callback
 *          to be called from dwt_isr() as an Rx call-back
 * */
void audio_rx_resp_cb(const dwt_cb_data_t *rxd)
{
    uwb_medium_access_t access;
    access.type = UWB_ACCESS_TX_FRAME_SENT;
    access.context.status = rxd->status;
    access.context.status_hi = rxd->status_hi;
    access.context.rx_flags = rxd->rx_flags;
    access.context.datalength = rxd->datalength;
    access.context.dss_stat = rxd->dss_stat;
#if UWB_RX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_rx_task_notify(AUDIO_RX_FRAME_NTF);
#else
    audio_rx_process_tx_frame_sent(access);
#endif
}
#endif

void audio_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
#if UWB_TRANS_DEBUG
    uwb_recieve_timeout++;
#endif
    uwb_medium_access_t access;
    access.type = UWB_ACCESS_RX_TIMEOUT;
    access.context.status = rxd->status;
    access.context.status_hi = rxd->status_hi;
    access.context.rx_flags = rxd->rx_flags;
    access.context.datalength = rxd->datalength;
    access.context.dss_stat = rxd->dss_stat;

#if UWB_RX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_rx_task_notify(AUDIO_RX_FRAME_NTF);
#else
    audio_rx_process_rx_timeout(access);
#endif
}

void audio_rx_error_cb(const dwt_cb_data_t *rxd)
{
#if UWB_TRANS_DEBUG
    uwb_receive_error++;
#endif
    uwb_medium_access_t access;
    access.type = UWB_ACCESS_RX_ERROR;
    access.context.status = rxd->status;
    access.context.status_hi = rxd->status_hi;
    access.context.rx_flags = rxd->rx_flags;
    access.context.datalength = rxd->datalength;
    access.context.dss_stat = rxd->dss_stat;

#if UWB_RX_USE_TASK
    audio_add_uwb_medium_access(access);
    audio_rx_task_notify(AUDIO_RX_FRAME_NTF);
#else
    audio_rx_process_rx_error(access);
#endif
}

//-----------------------------------------------------------------------------

/* @brief     app level
 *     RTOS-independent application level function.
 *     initializing of a Audio Rx functionality.
 *
 * */
error_e audio_rx_process_init()
{
    /* Configure non-zero initial variables.1 : from app parameters */

    /* The AudioRx has its configuration in the app->pConfig, see DEFAULT_CONFIG.
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

    /* Configure DW IC's UWB mode, sets power and antenna delays for Audio Rx mode
     * Configure SPI to fast rate */
    rf_tuning_t *rf_tuning = get_rf_tuning_config();
    audio_configure_uwb_rxtx(dwt_config,
                         DWT_FF_DATA_EN, /* No frame filtering for AudioRx */
                         rf_tuning->antTx_a,
                         rf_tuning->antRx_a);
#if UWB_TRANS_RETRY_ENABLE
    audio_uwb_init(audio_rx_resp_cb, audio_rx_done_cb, audio_rx_timeout_cb, audio_rx_error_cb, rf_tuning->xtalTrim);
#else
    audio_uwb_init(NULL, audio_rx_done_cb, audio_rx_timeout_cb, audio_rx_error_cb, rf_tuning->xtalTrim);
#endif

    /* End configuration of DW IC */

    AUDIO_EXIT_CRITICAL();

#if UWB_TRANS_RETRY_ENABLE && !UWB_RX_ACK_AUTO_ENABLE
    memset(rx_resp_buffer, 0x0, sizeof(rx_resp_buffer));
#if USB_RX_USE_DATA_FRAME_RESP
    rx_resp_buffer[0] = RESP_FC_0;
    rx_resp_buffer[1] = RESP_FC_1;
    rx_resp_buffer[2] = 0x0;
    rx_resp_buffer[3] = PAN_ID_LO;
    rx_resp_buffer[4] = PAN_ID_HI;
    rx_resp_buffer[5] = DEST_SHORT_ADDR_LO;
    rx_resp_buffer[6] = DEST_SHORT_ADDR_HI;
    rx_resp_buffer[7] = SHORT_ADDR_LO;
    rx_resp_buffer[8] = SHORT_ADDR_HI;
    rx_resp_buffer[9] = 0x0;
#else
    rx_resp_buffer[0] = RESP_FC_0;
    rx_resp_buffer[1] = RESP_FC_1;
    rx_resp_buffer[2] = 0x0;
#endif
#endif
    audio_buffer_init();

    return (_NO_ERR);
}

/*
 * @brief
 *     Enable DW3000 IRQ to start
 * */
void audio_rx_process_start(void)
{
    hal_uwb.enableIRQ();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* @brief     app level
 *     RTOS-independent application level function.
 *     deinitialize the pAudioInfo structure.
 *    This must be executed in protected mode.
 *
 * */
void audio_rx_process_deinit(void)
{
    rx_streaming_started = FALSE;
    rx_msg_sn = 0;
#if USE_PLL_CLOCK_AUDIO_SYNC 
    i2s_pll_num = I2S_PLL_DEFAULT_NUM_N;
    __HAL_RCC_PLLI2S_CONFIG(I2S_PLL_DEFAULT_NUM_N, I2S_PLL_DEFAULT_NUM_R);
#endif
    audio_uwb_deinit();
    audio_buffer_deinit();
}

/**
 * @description: Start audio streaming to DAC
 * @return {*}
 */
void audio_rx_process_trigger()
{
#if USE_I2S_AUDIO_PLAYBACK
    if (rx_streaming_started)
    {
        if (0 == audio_buffer_read_check(AUDIO_I2S_DATA_SIZE))
        {
            audio_info_t * info = getAudioInfoPtr();
            uint8_t *data_buffer = info->data_buffer->data;
            uint16_t current_rd_ptr = info->data_buffer->rd_ptr;
            audio_i2s_start(data_buffer + current_rd_ptr, AUDIO_I2S_DATA_SIZE);
            current_rd_ptr = (current_rd_ptr + AUDIO_I2S_DATA_SIZE) % info->data_buffer->size;
            audio_buffer_update_rd_ptr(current_rd_ptr);
        }
    }
#endif
}

/**
 * @description: Read out received audio sample to DAC when I2S DMA shift
 * @return {*}
 */
void audio_rx_data_out()
{
    audio_data_output_Call++;
    if (rx_streaming_started)
    {
#if USE_I2S_AUDIO_PLAYBACK
        if (0 == audio_buffer_read_check(AUDIO_I2S_DATA_SIZE))
        {
            audio_info_t * info = getAudioInfoPtr();
            uint8_t *data_buffer = info->data_buffer->data;
            uint16_t current_rd_ptr = info->data_buffer->rd_ptr;
            audio_i2s_update_buffer(data_buffer + current_rd_ptr, AUDIO_I2S_DATA_SIZE);
            current_rd_ptr = (current_rd_ptr + AUDIO_I2S_DATA_SIZE) % info->data_buffer->size;
            audio_buffer_update_rd_ptr(current_rd_ptr);
            audio_rx_sample_resync();
        }
        else
        {
            memset(i2s_alt_buffer, 0x0, sizeof(i2s_alt_buffer));
            audio_i2s_update_buffer(i2s_alt_buffer, sizeof(i2s_alt_buffer));
            audio_buffer_reset();
            rx_streaming_started = FALSE;
            rx_msg_sn = 0;
#if USE_PLL_CLOCK_AUDIO_SYNC
            i2s_pll_num = I2S_PLL_DEFAULT_NUM_N;
            __HAL_RCC_PLLI2S_CONFIG(I2S_PLL_DEFAULT_NUM_N, I2S_PLL_DEFAULT_NUM_R);
#endif
#if UWB_TRANS_DEBUG
            audio_data_output_error++;
#endif
        }
#elif USE_USB_AUDIO_RECORDING

        if (0 == audio_buffer_fetch_data(timer_data_buffer, getAudioInfoPtr()->ms_audio_sample_size))
        {
            UWB_AUDIO_IN_TransferComplete_CallBack(timer_data_buffer, getAudioInfoPtr()->ms_audio_sample_size);
            audio_rx_sample_resync();
        }
        else
        {
            audio_buffer_reset();
            rx_streaming_started = FALSE;
            rx_msg_sn = 0;
            memset(timer_data_buffer, 0x0, sizeof(timer_data_buffer));
            timer_period = TIMER_SYNC_DEFAULT_PERIOD;
#if UWB_TRANS_DEBUG
            audio_data_output_error++;
#endif
        }
#endif
    }
    else
    {
    #if USE_I2S_AUDIO_PLAYBACK
        audio_i2s_pause();
    #elif USE_USB_AUDIO_RECORDING
        UWB_AUDIO_IN_TransferComplete_CallBack(timer_data_buffer, getAudioInfoPtr()->ms_audio_sample_size);
    #endif
    }
}

void audio_rx_update_package_size(uint16_t ms_audio_sample_size)
{
    audio_update_package_size(ms_audio_sample_size);
}

uint16_t audio_rx_get_sync_value()
{
    if (rx_streaming_started)
    {
        return getAudioInfoPtr()->data_buffer->rd_ptr;
    }
    return 0;
}

uint16_t audio_rx_get_buffer_size()
{
    return AUDIO_UWB_BUFFER_SIZE;
}

/**
 * @description: Adjust I2S clack according to UWB receiving package buffer size
 * @return {*}
 */
void audio_rx_sample_resync(void)
{
#if USE_I2S_AUDIO_PLAYBACK
    uint16_t data_size = audio_buffer_data_size();
    uint16_t buffer_size_half = getAudioInfoPtr()->data_buffer->size / 2;
    uint16_t new_i2s_pll_num = i2s_pll_num;

    // Slightly adjust I2S PLL to speed up or down audio consumption to adapt the sample rate of UWB TX side
    if (data_size > (buffer_size_half + I2S_PLL_ADJUST_THRESHOLD))
    {
        if(i2s_pll_num <= I2S_PLL_DEFAULT_NUM_N)
		{
			new_i2s_pll_num = i2s_pll_num + 1;
		}
    }
    else if (data_size < (buffer_size_half - I2S_PLL_ADJUST_THRESHOLD))
    {
        if(i2s_pll_num >= I2S_PLL_DEFAULT_NUM_N)
		{
			new_i2s_pll_num = i2s_pll_num - 1;
		}
    }
    else if (data_size == buffer_size_half)
    {
        new_i2s_pll_num = I2S_PLL_DEFAULT_NUM_N;
    }
    
    if(new_i2s_pll_num != i2s_pll_num)
    {
#if USE_PLL_CLOCK_AUDIO_SYNC
        __HAL_RCC_PLLI2S_CONFIG(new_i2s_pll_num,I2S_PLL_DEFAULT_NUM_R);
#else
        if (new_i2s_pll_num > I2S_PLL_DEFAULT_NUM_N)
        {
            i2s_pll_clock_drift_flag = I2S_AUDIO_OVERRUN;
        }
        else if (new_i2s_pll_num < I2S_PLL_DEFAULT_NUM_N)
        {
            i2s_pll_clock_drift_flag = I2S_AUDIO_UNDERRUN;
        }
        else
        {
            i2s_pll_clock_drift_flag = I2S_AUDIO_NORMAL;
        }
#endif
        i2s_pll_num = new_i2s_pll_num;
#if UWB_TRANS_DEBUG
        audio_i2s_pll_updated++;
#endif
    }
#elif USE_USB_AUDIO_RECORDING
    uint16_t data_size = audio_buffer_data_size();

    if (data_size > getAudioInfoPtr()->buffer->size / 2)
    {
        timer_period = timer_period - TIMER_SYNC_STEP;
    }
    else if (data_size < getAudioInfoPtr()->buffer->size / 2)
    {
        timer_period = timer_period + TIMER_SYNC_STEP;
    }
    else
    {
        timer_period = TIMER_SYNC_DEFAULT_PERIOD;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim4, timer_period);
#endif
}

static uint16_t debug_receive_ok[10] = {0};
static uint16_t debug_receive_err[10] = {0};
static uint16_t debug_receive_to[10] = {0};
static uint32_t debug_audio_i2s_call[10] = {0};
static uint16_t debug_index = {0};


void audio_rx_debug_output()
{
#if UWB_TRANS_DEBUG
    if (uwb_receive_ok || uwb_receive_error
    || uwb_recieve_timeout || uwb_receive_msg_error
    || uwb_receive_msg_retry || fill_buffer_error
    || fetch_buffer_error || audio_data_output_error)
    {
        debug_receive_ok[debug_index] = uwb_receive_ok;
        debug_receive_err[debug_index] = uwb_receive_error;
        debug_receive_to[debug_index] = uwb_receive_msg_error;
        debug_audio_i2s_call[debug_index] = audio_data_output_Call;

        if (++debug_index > 9)
        {
            debug_index = 0;
        }
        // STM32CubeIDE Breakpoint log: "%dR%dE%dT%dS-Buffer:%d-%d-Sample:%d\r\n",  uwb_receive_ok, uwb_receive_error, uwb_recieve_timeout, uwb_receive_msg_error, fill_buffer_error, fetch_buffer_error, audio_data_output_error
        // VSCode Breakpoint log: "%dR%dE%dT%dS-Buffer:%d-%d-Sample:%d-Resp:%d\r\n" uwb_receive_ok uwb_receive_error uwb_recieve_timeout uwb_receive_msg_error fill_buffer_error fetch_buffer_error audio_data_output_error uwb_receive_msg_retry
#if !USE_USB_AUDIO_PLAYBACK && !USE_USB_AUDIO_RECORDING
        sprintf(debug_log_buffer, "\"R:%5d\",\"E:%5d\",\"T:%5d\",\"S:%5d\",\"Buffer:%5d-%5d\",\"Retry:%5d-%5d\",\"I2s:%5d\"\r\n", uwb_receive_ok, 
        uwb_receive_error, uwb_recieve_timeout, uwb_receive_msg_error, fill_buffer_error, fetch_buffer_error,
        uwb_receive_msg_retry, audio_data_output_error, audio_i2s_pll_updated);
        copy_tx_msg((uint8_t*)debug_log_buffer, strlen(debug_log_buffer));
#endif
        uwb_receive_ok          = 0;
        uwb_receive_error       = 0;
        uwb_recieve_timeout     = 0;
        uwb_receive_msg_error   = 0;
        uwb_receive_msg_retry   = 0;
        fill_buffer_error       = 0;
        fetch_buffer_error      = 0;
        audio_data_output_error = 0;
        audio_i2s_pll_updated   = 0;
        audio_data_output_Call = 0;
    }
#endif
}
//-----------------------------------------------------------------------------
