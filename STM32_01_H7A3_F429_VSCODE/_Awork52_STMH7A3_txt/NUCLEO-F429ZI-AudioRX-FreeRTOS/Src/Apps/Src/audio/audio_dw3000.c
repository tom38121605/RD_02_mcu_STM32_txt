/**
 * @file      audio_dw3000.c
 * 
 * 
 * @brief     Decawave Application level
 *             collection of DW3000 bare-metal functions for audio
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
#include <assert.h>
#include "deca_device_api.h"
#include "driver_app_config.h"
#include "rf_tuning_config.h"
#include "common_audio.h"

#if UWB_TRANS_RETRY_ENABLE
/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). */
#if UWB_RX_ACK_AUTO_ENABLE
#define TX_TO_RX_DELAY_UUS 0
#endif
/* Receive response timeout for TX, expressed in UWB microseconds. */
#define TX_RESP_TO_UUS 400
#endif
/* Receive response timeout, expressed in UWB microseconds. */
#define RX_RESP_TO_UUS 5000

/*
 * TX Power Configuration Settings
 */
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. */
dwt_txconfig_t txconfig_options = {
    0x34,       /* PG delay. */
#if USE_AUDIO_TX
    0xfdfdfdfd, /* TX power. */
#elif USE_AUDIO_RX
    0xfdfdfdfd, /* TX power. */
#endif
    0x0         /*PG count*/
};

void audio_configure_uwb_rxtx(
    dwt_config_t *pdwCfg,
    uint16_t frameFilter,
    uint16_t txAntDelay,
    uint16_t rxAntDelay)
{

#if UWB_USE_DW3000_SCP_MODE
    pdwCfg->chan           = 5;
    pdwCfg->txPreambLength = DWT_PLEN_64;
    pdwCfg->rxPAC          = DWT_PAC8;
    pdwCfg->txCode         = 25;
    pdwCfg->rxCode         = 25;
    pdwCfg->sfdType        = DWT_SFD_IEEE_4Z;
    pdwCfg->dataRate       = DWT_BR_6M8;
    pdwCfg->phrMode        = DWT_PHRMODE_EXT;
    pdwCfg->phrRate        = DWT_PHRRATE_STD;
    pdwCfg->sfdTO          = (64 + 1 + 8 - 8);
    pdwCfg->stsMode        = DWT_STS_MODE_1 | DWT_STS_MODE_SDC;
    pdwCfg->stsLength      = DWT_STS_LEN_64;
    pdwCfg->pdoaMode       = DWT_PDOA_M0;
#else
    pdwCfg->chan           = 5;
    pdwCfg->txPreambLength = DWT_PLEN_128;
    pdwCfg->rxPAC          = DWT_PAC8;
    pdwCfg->txCode         = 9;
    pdwCfg->rxCode         = 9;
    pdwCfg->sfdType        = 1;
    pdwCfg->dataRate       = DWT_BR_6M8;
    pdwCfg->phrMode        = DWT_PHRMODE_EXT;
    pdwCfg->phrRate        = DWT_PHRRATE_STD;
    pdwCfg->sfdTO          = (129 + 8 - 8);
    pdwCfg->stsMode        = DWT_STS_MODE_OFF;
    pdwCfg->stsLength      = DWT_STS_LEN_64;
    pdwCfg->pdoaMode       = DWT_PDOA_M0;
#endif

    /**< Configure the Physical Channel parameters (PLEN, PRF, etc) */
    if (dwt_configure(pdwCfg))
    {
        error_handler(1, _ERR_INIT);
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
#if UWB_DATA_TRANS_DEBUG_TX || UWB_DATA_TRANS_DEBUG_RX
    dwt_txconfig_t *txConfig = get_dwt_txconfig();
    dwt_configuretxrf(txConfig);
#else
    dwt_configuretxrf(&txconfig_options);
#endif

    /* Set PAN ID and short address */
    uint16_t pan_id = (PAN_ID_HI << 8) + PAN_ID_LO;
    uint16_t short_addr = (SHORT_ADDR_HI << 8) + SHORT_ADDR_LO;
    dwt_setpanid(pan_id);
    dwt_setaddress16(short_addr);

    /* Configure frame filtering. Only data frames are enabled in this example. Frame filtering must be enabled for Auto ACK to work. */
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, frameFilter);


#if UWB_RX_AUTO_ENABLE
    dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS, DBL_BUF_MODE_AUTO); /* Disable double buffer - auto RX re-enable mode */
#else
#if UWB_RX_DB_ENABLE
    dwt_setdblrxbuffmode(DBL_BUF_STATE_EN, DBL_BUF_MODE_MAN); /* Enable double buffer - manual RX re-enable mode */
#else
    dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS, DBL_BUF_MODE_MAN); /* Disable double buffer - manual RX re-enable mode */
#endif
#endif

#if USE_AUDIO_RX
    dwt_setrxtimeout(RX_RESP_TO_UUS); /* Set RX timeout for audio receiver side */
    
    /* Activate auto-acknowledgement. */
#if UWB_RX_ACK_AUTO_ENABLE
    dwt_enableautoack(0, 1);
#else
    dwt_enableautoack(0, 0);
#endif
#endif

#if USE_USB_AUDIO_PLAYBACK || UWB_DATA_TRANS_DEBUG_TX
#if UWB_TRANS_RETRY_ENABLE
    /* If use data frame to send response, transmitter need to turn on antenna manually */
#if UWB_RX_ACK_AUTO_ENABLE
    /* Set delay to turn reception on after transmission of the frame. */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);
#endif
    /* Set response frame timeout for audio sender. */ 
    dwt_setrxtimeout(TX_RESP_TO_UUS);
#endif
#endif

}

void audio_uwb_init(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr, uint8_t xtalTrim)
{
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;     /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_PA_ENABLE | DWT_LNA_ENABLE);        /**< DEBUG I/O 5&6 : configure TX/RX states to output on GPIOs */

    dwt_setcallbacks(cbTxDone, cbRxOk, cbRxTo, cbRxErr, NULL, NULL, NULL);

    dwt_setinterrupt( DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK
                    | (DWT_INT_ARFE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK
                            | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFTO_BIT_MASK /*| SYS_STATUS_RXPTO_BIT_MASK*/), 0, 2);

}

void audio_uwb_deinit(void)
{
    //DW_IRQ is disabled: safe to cancel all user call-backs
    dwt_setcallbacks(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}