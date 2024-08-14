/*
 * Copyright (c) 2022 Qorvo, Inc
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

#ifndef __LLHW_DIAG_H__
#define __LLHW_DIAG_H__

#include "llhw.h"

/**
 * llhw_compute_rssi() - Compute an RSSI estimate
 * @uwb: the UWB chip.
 * @samples: the RSSI samples obtained during RX.
 *
 * Return: 0 on error, else the RSSI in absolute value and as an integer.
 * expressed in dBm, Q32.0.
 */
uint32_t llhw_compute_rssi(const struct uwb_chip *uwb,
			   const struct uwb_rssi_samples *samples);

/**
 * llhw_read_diagnostics() - Read diagnostics
 * @uwb: the UWB chip.
 * @rx: the RX frame structure.
 * @info: the RX measurement informations.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_read_diagnostics(const struct uwb_chip *uwb, struct uwb_rx *rx,
			  struct mcps802154_rx_measurement_info *info);

#endif /* __LLHW_DIAG_H__ */
