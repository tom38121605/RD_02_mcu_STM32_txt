/**
 *  @file     common_audio.h
 *
 *  @brief    Header file for common_audio
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

#ifndef CORE_SRC_APPS_COMMON_AUDIO_H_
#define CORE_SRC_APPS_COMMON_AUDIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "HAL_error.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "deca_error.h"
#include "apps_common.h"
#include "user_audio_config.h"

// Maximum payload size for one data package
// 802.15.4z UWB standard maximum frame length is 127 bytes. DW IC also supports an extended frame length (up to 1023 bytes long) mode.
#define FRAME_LEN_MAX    (127)
#define FRAME_LEN_MAX_EX (1023)

// implementation-specific: critical section protection
#ifndef AUDIO_ENTER_CRITICAL
#define AUDIO_ENTER_CRITICAL enter_critical_section
#endif

#ifndef AUDIO_EXIT_CRITICAL
#define AUDIO_EXIT_CRITICAL leave_critical_section
#endif

/* Index to access the header of DATA frame */
#if USB_RX_USE_DATA_FRAME_RESP
#define FRAME_FC_0 0x41
#else
#define FRAME_FC_0 0x61
#endif
#define FRAME_FC_1 0x88
#define FRAME_FC_IDX 0
#define FRAME_SN_IDX 2
#define FRAME_DATA_INDEX 9

#if USB_RX_USE_DATA_FRAME_RESP
/* DATA frame response control value. */
#define RESP_FC_0 0x41
#define RESP_FC_1 0x88
#define RESP_FC_IDX 0
#define RESP_SN_IDX 2
#define RESP_DATA_INDEX 9
#define RESP_FRAME_LEN 12
#else
/* ACK frame control value. */
#define RESP_FC_0 0x02
#define RESP_FC_1 0x00
#define RESP_SN_IDX 2
#define RESP_FRAME_LEN 5
#endif

#define ACK_FC_0 0x02     //--add
#define ACK_FC_1 0x00     //--add
#define ACK_FRAME_LEN 5   //--add

/* PAN ID/short address. 
 * PAN ID and short address are hard coded constants to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the DW IC
 *    during its manufacture. However there is no guarantee this will not conflict with someone else implementation. We recommended that customers
 *    buy a block of addresses from the IEEE Registration Authority for their production items.
 * */
// #define PAN_ID     0xDECA
#define PAN_ID_HI     0xAD
#define PAN_ID_LO     0xAD

#if USE_AUDIO_RX
// #define SHORT_ADDR 0x5258 /* "RX" */
#define SHORT_ADDR_HI 0x52
#define SHORT_ADDR_LO 0x58

#define DEST_SHORT_ADDR_HI 0x54
#define DEST_SHORT_ADDR_LO 0x58
#endif
#if USE_AUDIO_TX
// #define SHORT_ADDR 0x5458 /* "TX" */
#define SHORT_ADDR_HI 0x54
#define SHORT_ADDR_LO 0x58

#define DEST_SHORT_ADDR_HI 0x52
#define DEST_SHORT_ADDR_LO 0x58
#endif

#define AUDIO_FRAME_MAX_DATA_SIZE (FRAME_LEN_MAX_EX - FCS_LEN - FRAME_DATA_INDEX)

/* AUDIO_MS_PACKET_SIZE compute the nominal size(number of bytes) of an audio packet requierd for one millisecond
 * , for example for audio 48KHZ/24 bit/sterio required size is 48*3*2 , for 44.1KHZ/16bits/sterio required size is 44*2*2 */
#define AUDIO_UWB_MS_PACKET_SIZE(freq,channel_count,res_byte) (((uint32_t)((freq) /1000))* (channel_count) * (res_byte)) 

#define AUDIO_RESOLUTION_BYTE AUDIO_CONFIG_RES_BYTE
#define AUDIO_MS_SAMPLE_SIZE ((uint16_t)(AUDIO_UWB_MS_PACKET_SIZE((AUDIO_CONFIG_FREQ_MAX), AUDIO_CONFIG_CHANNEL_COUNT, AUDIO_CONFIG_RES_BYTE)))
#define AUDIO_UWB_PACKAGE_SIZE (AUDIO_FRAME_MAX_DATA_SIZE - (AUDIO_FRAME_MAX_DATA_SIZE % (AUDIO_RESOLUTION_BYTE * AUDIO_CONFIG_CHANNEL_COUNT)))

#if USE_FEEDBACK_AUDIO_SYNC
#define AUDIO_FEEDBACK_SYNC_OFFSET_STEP 20
#endif

typedef enum
{
  I2S_AUDIO_NORMAL = 0,
  I2S_AUDIO_UNDERRUN = 1,
  I2S_AUDIO_OVERRUN = 2
} i2s_audio_feedback_sync_flag;

#if USE_I2S_AUDIO_PLAYBACK
#if USE_AUDIO_RX
#define AUDIO_I2S_DATA_SIZE  (AUDIO_UWB_PACKAGE_SIZE / 2)
#define I2S_PLL_ADJUST_THRESHOLD (AUDIO_UWB_PACKAGE_SIZE * 2)
#elif USE_AUDIO_TX
#define AUDIO_I2S_DATA_SIZE  (AUDIO_MS_SAMPLE_SIZE)
#endif
#endif

/* Maximum TX retransmission attempts if ACK and retry enabled*/
#define UWB_TX_ATTEMPT_TIMES 5
/* The UWB RX audio buffer size. The buffer delay should be AUDIO_UWB_BUFFER_SIZE/AUDIO_MS_SAMPLE_SIZE/2.
 * Audio streaming will start when buffer is half full, which means the actual audio delay is
 * the half of buffer size.
 */
#define AUDIO_UWB_BUFFER_SIZE (AUDIO_UWB_PACKAGE_SIZE * 10)

#define UWB_MEDIUM_ACCESS_BUFFER_SIZE (10)

//-----------------------------------------------------------------------------

// UWB data transmission buffer
struct audio_data_circular_buffer_s
{
  uint8_t *data;   /* pointer to circular buffer data */
  uint16_t rd_ptr; /* circular buffer reading offset */
  uint16_t wr_ptr; /* circular buffer writing offset */
  uint16_t size;   /* The size of buffer segment where samples may be read or written. It is equal or less than the real size of the buffer  */
};

typedef struct audio_data_circular_buffer_s audio_data_circular_buffer_t;

struct uwb_access_context_s
{
    uint32_t status;     // initial value of register as ISR is entered
    uint16_t status_hi;  // initial value of register as ISR is entered, if relevant for that event type
    uint16_t datalength; // length of frame
    uint8_t * data;      // data buffer of frame
    uint8_t  rx_flags;   // RX frame flags, see above
    uint8_t  dss_stat;   // Dual SPI status reg 11:38, 2 LSbits relevant : bit0 (DWT_CB_DSS_SPI1_AVAIL) and bit1 (DWT_CB_DSS_SPI2_AVAIL)
};

typedef struct uwb_access_context_s uwb_access_context_t;

typedef enum
{
  UWB_ACCESS_INVALID = 0,
  UWB_ACCESS_TX_FRAME_START,
  UWB_ACCESS_TX_FRAME_SENT,
  UWB_ACCESS_RX_FRAME,
  UWB_ACCESS_RX_TIMEOUT,
  UWB_ACCESS_RX_ERROR
} uwb_medium_access_type;

// UWB transmission access holder
struct uwb_medium_access_s
{
  uwb_medium_access_type type;
  uwb_access_context_t context;
};

typedef struct uwb_medium_access_s uwb_medium_access_t;

// UWB transmission infomation holder
struct audio_info_s
{
  audio_data_circular_buffer_t *data_buffer;
  uwb_medium_access_t *access_buffer;
  uint16_t next_access_index;
  uint16_t last_access_index;
  uint16_t ms_audio_sample_size;
  uint16_t package_size;
};

typedef struct audio_info_s audio_info_t;

//-----------------------------------------------------------------------------
// HW-specific function implementation
//
void audio_uwb_init(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr, uint8_t xtalTrim);
void audio_uwb_deinit(void);

//-----------------------------------------------------------------------------
// exported functions prototypes
//
extern audio_info_t * getAudioInfoPtr(void);
void audio_update_package_size(uint16_t ms_audio_sample_size);
void audio_configure_uwb_rxtx(dwt_config_t *pdwCfg, uint16_t frameFilter, uint16_t txAntDelay, uint16_t rxAntDelay);
void audio_wait_for_sys_status(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask);

//-----------------------------------------------------------------------------
// exported functions prototypes for buffer handling
//
int audio_buffer_init(void);
/**
 * @description: 
 * @return {*}
 */
int audio_buffer_deinit(void);
int audio_buffer_write_check(uint16_t data_size);
int audio_buffer_read_check(uint16_t data_size);
int audio_buffer_data_size();
int audio_buffer_fill_data(uint8_t *data_buffer, uint16_t data_size);
int audio_buffer_fetch_data(uint8_t *data_buffer, uint16_t data_size);
void audio_buffer_update_rd_ptr(uint16_t new_rd_ptr);
void audio_buffer_update_wr_ptr(uint16_t new_wr_ptr);
void audio_buffer_reset(void);
int audio_add_uwb_medium_access(const uwb_medium_access_t access);
int audio_get_uwb_medium_access(uwb_medium_access_t * access);
#ifdef __cplusplus
}
#endif

#endif /* CORE_SRC_APPS_COMMON_AUDIO_H_ */
