/**
 *  @file     user_audio_config.h
 *
 *  @brief    Header file for user_audio_config
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

#ifndef CORE_SRC_APPS_USER_AUDIO_CONFIG_H_
#define CORE_SRC_APPS_USER_AUDIO_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "audio_constants.h"

#define AUDIO_CONFIG_CHANNEL_COUNT          0x02 /* stereo audio  */
#define AUDIO_CONFIG_CHANNEL_MAP            0x03 /* channels Left and right */
/* next two values define the supported resolution  currently expansion supports only 16 bit and 24 bits resolutions @TODO add other resolution support*/
#define AUDIO_CONFIG_RES_BIT                24 /* 16 bit per sample */
#define AUDIO_CONFIG_RES_BYTE               3 /* 2 bytes */   
/* definition of the list of frequencies */
#define AUDIO_CONFIG_USE_FREQ_192_K          0 /* to set by user:  1 : to use , 0 to not support*/
#define AUDIO_CONFIG_USE_FREQ_96_K           1 /* to set by user:  1 : to use , 0 to not support*/
#define AUDIO_CONFIG_USE_FREQ_48_K           0 /* to set by user:  1 : to use , 0 to not support*/
#define AUDIO_CONFIG_USE_FREQ_44_1_K         0 /* to set by user:  1 : to use , 0 to not support*/
#define AUDIO_CONFIG_USE_FREQ_32_K           0 /* to set by user:  1 : to use , 0 to not support*/
#define AUDIO_CONFIG_USE_FREQ_16_K           0 /* to set by user:  1 : to use , 0 to not support*/
#define AUDIO_CONFIG_USE_FREQ_8_K            0 /* to set by user:  1 : to use , 0 to not support*/

/*playback computing the max and the min frequency */  
#if AUDIO_CONFIG_USE_FREQ_192_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_192_K
#elif AUDIO_CONFIG_USE_FREQ_96_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_96_K
#elif AUDIO_CONFIG_USE_FREQ_48_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_48_K
#elif AUDIO_CONFIG_USE_FREQ_44_1_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_44_1_K
#elif AUDIO_CONFIG_USE_FREQ_32_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_32_K
#elif AUDIO_CONFIG_USE_FREQ_16_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_16_K
#elif AUDIO_CONFIG_USE_FREQ_8_K
#define AUDIO_CONFIG_FREQ_MAX   AUDIO_CONFIG_FREQ_8_K
#else
#error "Playback frequency is missed"
#endif 

#if AUDIO_CONFIG_USE_FREQ_8_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_8_K
#elif AUDIO_CONFIG_USE_FREQ_16_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_16_K
#elif AUDIO_CONFIG_USE_FREQ_32_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_32_K
#elif AUDIO_CONFIG_USE_FREQ_44_1_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_44_1_K
#elif AUDIO_CONFIG_USE_FREQ_48_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_48_K
#elif AUDIO_CONFIG_USE_FREQ_96_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_96_K
#elif AUDIO_CONFIG_USE_FREQ_192_K
#define AUDIO_CONFIG_FREQ_MIN   AUDIO_CONFIG_FREQ_192_K
#endif 

/* Turn on off UWB transmission debug output to USB-UART */
#define UWB_TRANS_DEBUG 1
/* Turn on off UWB transmission ACK and retry */
#define UWB_TRANS_RETRY_ENABLE 1
/* UWB RX will auto enable antenna when a frame reception failure (except a frame wait timeout) */
#define UWB_RX_AUTO_ENABLE 1
/* UWB RX will use double buffer mode. Notes, RX antenna auto enable cannot work if double buffer enabled */
#define UWB_RX_DB_ENABLE 0

#if UWB_TRANS_RETRY_ENABLE
/* UWB RX will send ACK when a good frame received */
#define UWB_RX_ACK_AUTO_ENABLE 1

#if !UWB_RX_ACK_AUTO_ENABLE
/* Send out response using data frame, not act frame*/
#define USB_RX_USE_DATA_FRAME_RESP 1
#endif

#endif

/* UWB RX side can handle received packages in interrupt or FreeRTOS task */
#define UWB_RX_USE_TASK 0
/* UWB TX side can handle transmission in interrupt or FreeRTOS task */
#define UWB_TX_USE_TASK 0
/* Enable DW3000's Short Compress Mode which enable 27Mbps PHY  */
#define UWB_USE_DW3000_SCP_MODE 1

#if USE_I2S_AUDIO_PLAYBACK

/* Set I2S buffer to process in interrupt or FreeRTOS task */
#if USE_AUDIO_TX
#define I2S_BUFFER_USE_TASK 1
#endif
#if USE_AUDIO_RX
#define I2S_BUFFER_USE_TASK 1
#endif
/* Adjust PLL clock at receiver side to adapt the sample rate of transmitter.
** If DISABLE, a feedback audio sync must implement in transmitter. When Disable,
** UWB_RX_AUTO_ENABLE and UWB_RX_ACK_AUTO_ENABLE cannot enable.*/
#define USE_PLL_CLOCK_AUDIO_SYNC 0
#define USE_FEEDBACK_AUDIO_SYNC (!UWB_RX_AUTO_ENABLE && !UWB_RX_ACK_AUTO_ENABLE && !USE_PLL_CLOCK_AUDIO_SYNC)

/* Adjust default I2S PLL clock settings */
#if AUDIO_CONFIG_USE_FREQ_96_K
#define I2S_PLL_DEFAULT_NUM_N 430
#define I2S_AUDIOFREQ_HZ I2S_AUDIOFREQ_96K
#else
#define I2S_PLL_DEFAULT_NUM_N 384
#define I2S_AUDIOFREQ_HZ I2S_AUDIOFREQ_48K
#endif
#if AUDIO_CONFIG_RES_BIT == 24
#define I2S_DATAFORMAT_BIT I2S_DATAFORMAT_24B
#else
#define I2S_DATAFORMAT_BIT I2S_DATAFORMAT_16B
#endif
#define I2S_PLL_DEFAULT_NUM_R 5
#endif

#ifdef __cplusplus
}
#endif

#endif /* CORE_SRC_APPS_USER_AUDIO_CONFIG_H_ */
