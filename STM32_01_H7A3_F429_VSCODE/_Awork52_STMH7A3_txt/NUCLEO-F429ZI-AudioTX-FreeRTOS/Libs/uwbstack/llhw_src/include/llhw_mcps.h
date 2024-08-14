/*
 * Copyright (c) 2021-2022 Qorvo, Inc
 *
 * All rights reserved.
 *
 * NOTICE: All information contained herein is, and remains the property
 * of Qorvo, Inc. and its suppliers, if any. The intellectual and technical
 * concepts herein are proprietary to Qorvo, Inc. and its suppliers, and
 * may be covered by patents, patent applications, and are protected by
 * trade secret and/or copyright law. Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written
 * permission is obtained from Qorvo, Inc.
 *
 */

#ifndef __LLHW_MCPS_H__
#define __LLHW_MCPS_H__

#include "llhw.h"

/**
 * llhw_mcps_init() - initialize and allocate a new low-level hardware device.
 * @uwb: the UWB device to initialize.
 *
 * Return: the allocated Low-level hardware without MCPS.
 */
struct mcps802154_llhw *llhw_mcps_init(struct uwb_chip *uwb);

/**
 * llhw_mcps_init() - deinitialize and deallocate the low-level hardware device.
 */
void llhw_mcps_deinit(void);

/**
 * llhw_mcps_rx_cb() - callback that handles RX good frame event.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_rx_cb(const dwt_cb_data_t *rxd);

/**
 * llhw_mcps_rxerror_cb() - callback that handles RX error frame event.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_rxerror_cb(const dwt_cb_data_t *rxd);

/**
 * llhw_mcps_rxtimeout_cb() - callback that handles RX timeout frame event.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_rxtimeout_cb(const dwt_cb_data_t *rxd);

/**
 * llhw_mcps_txdone_cb() - callback that handles TX done frame event.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_txdone_cb(const dwt_cb_data_t *txd);

/**
 * llhw_mcps_timerexpired_cb() - callback that handles timer expiration event.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_timerexpired_cb(const dwt_cb_data_t *data);

/**
 * llhw_mcps_pctt_loopback_rx_cb() - specific callback that handles RX error
 * frame event for Loopback test only.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_pctt_loopback_rx_cb(const dwt_cb_data_t *cb_data);

/**
 * llhw_mcps_pctt_loopback_rxerror_cb() - specific callback that handles RX error
 * or timeout frame event for Loopback test only.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_pctt_loopback_rxerror_cb(const dwt_cb_data_t *cb_data);

/**
 * llhw_mcps_pctt_loopback_txdone_cb() - specific callback that handles TX done
 * frame event for Loopback test only.
 * @rxd: TX/RX call-back data.
 */
void llhw_mcps_pctt_loopback_txdone_cb(const dwt_cb_data_t *cb_data);

/**
 * llhw_mcps_call_wakeup_callback() - call wake-up callback.
 * @dss: deep-sleep informations.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_mcps_call_wakeup_callback(struct uwb_deep_sleep_state *dss);

/**
 * llhw_mcps_rxtimeout() - handles RX timeout frame event (triggered from timer
 * configured in software).
 * @uwb: the UWB device.
 */
void llhw_mcps_rxtimeout(struct uwb_chip *uwb);

#endif /* __LLHW_MCPS_H__ */
