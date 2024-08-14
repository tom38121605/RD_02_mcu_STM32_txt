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

#ifndef __LLHW_TIME_H__
#define __LLHW_TIME_H__

#include "llhw.h"

/**
 * llhw_get_dtu_time() - get current DTU time
 * @uwb: the UWB device.
 *
 * This function compute the current DTU time, based on "UWB" time,
 * at 15.6MHz rate.
 *
 * Return: the current DTU time
 */
uint32_t llhw_get_dtu_time(const struct uwb_chip *uwb);

/**
 * llhw_systime_to_dtu() - compute current DTU time from SysTime.
 * @uwb: the UWB device.
 * @systime: the UWB transceiver SysTime register value to convert to DTU.
 * @dtu_near: a DTU time which must be in the past relative to systime, at less
 * than half the SysTime rollover period.
 *
 * Return: the corresponding DTU time
 */
uint32_t llhw_systime_to_dtu(const struct uwb_chip *uwb, uint32_t systime,
			     uint32_t dtu_near);

/**
 * llhw_systime_rctu_to_dtu() - compute current DTU time from RCTU.
 * @uwb: the UWB device.
 * @timestamp_rctu: The UWB transceiver RX_STAMP register value in RCTU to
 * convert to DTU.
 * The RCTU, Ranging Counter Time Unit, is approximately 15.65 picoseconds long.
 *
 * Return: The corresponding DTU time.
 */
uint32_t llhw_systime_rctu_to_dtu(const struct uwb_chip *uwb,
				  uint64_t timestamp_rctu);

/**
 * llhw_resync_dtu_systime() - resync DTU time and SysTime
 * @uwb: the UWB device.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_resync_dtu_systime(struct uwb_chip *uwb);

/**
 * llhw_time_us_to_dtu() - convert UWB time in microseconds to DTU time
 * @runtime: the Low-level hardware runtime instance
 * @timestamp_us: OS time in us
 *
 * Formula:
 * dtu = (time - time0) * N / D + dtu0
 * Where:
 * N = DTU_FREQ = 15600000
 * D = 1000000
 *
 * N/D = 15600000/1000000 = 156/10
 * dtu0 always 0.
 *
 * Return: driver DTU time
 */
static inline uint32_t llhw_time_us_to_dtu(const struct uwb_runtime *runtime,
					   int64_t timestamp_us)
{
	timestamp_us -= runtime->time_zero_us;
	return (u32)(timestamp_us * (DTU_FREQ / 100000) / 10);
}

/**
 * llhw_dtu_to_systime() - compute UWB SysTime from DTU time
 * @runtime: the Low-level hardware runtime instance
 * @dtu: the DTU timestamp to convert to SysTime
 *
 * Return: the value to write to SysTime register
 */
static inline u32 llhw_dtu_to_systime(const struct uwb_runtime *runtime,
				      uint32_t dtu)
{
	const int N = DTU_PER_SYS_POWER;
	u32 dtu_sync = runtime->dtu_sync;
	u32 systime_sync = runtime->systime_sync;
	return ((dtu - dtu_sync) << N) + systime_sync;
}

#endif /* __LLHW_TIME_H__ */
