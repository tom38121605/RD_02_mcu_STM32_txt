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

#ifndef __LLHW_DRV__H__
#define __LLHW_DRV__H__

#include "llhw.h"

#define IEEE802154_FCS_LEN 2

#define TARGET_XTAL_OFFSET_VALUE_PPHM_MAX (700)
#define TARGET_XTAL_OFFSET_VALUE_PPHM_MIN (500)
/* Trimming per 1 pphm*/
#define AVG_TRIM_PER_PPHM ((XTAL_TRIM_BIT_MASK + 1) / 48.0f / 100)

/* This define needs to be removed when PEG is fixed */
#define OTP_QM35B0_SIP_DEVICE 0x80000000UL
/**
 * llhw_drv_init() - Initialize LLHW driver.
 * @uwb: the UWB device.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_drv_init(struct uwb_chip *uwb);

/**
 * llhw_drv_reinit() - Reinitialize UWB chip.
 * @uwb: the UWB device.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_drv_reinit(const struct uwb_chip *uwb);

/**
 * llhw_drv_setup_coex_timer() - Configure timer used for WiFi Coex.
 * @uwb: the UWB device.
 * @delay_us: expiration delay in microseconds.
 */
void llhw_drv_setup_coex_timer(const struct uwb_chip *uwb, uint32_t delay_us);

/**
 * llhw_drv_get_nb_receivers() - Get number of receivers configured.
 * @uwb: the UWB device.
 *
 * Return: number of receivers.
 */
int llhw_drv_get_nb_receivers(const struct uwb_chip *uwb);

/**
 * llhw_drv_read_rx_timestamp() - Read RX frame timestamp.
 * @uwb: the UWB device.
 * @ts: the returned timestamp.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_drv_read_rx_timestamp(const struct uwb_chip *uwb, uint64_t *ts);

/**
 * llhw_drv_set_channel() - Set UWB channel.
 * @uwb: the UWB device.
 * @channel: channel.
 * @preamble_code: preamble code.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_drv_set_channel(struct uwb_chip *uwb, uint8_t channel,
			 uint8_t preamble_code);

/**
 * llhw_drv_get_time_us() - get UWB time in microseconds.
 * @uwb: the UWB device.
 *
 * Return: the current time in microseconds.
 */
int64_t llhw_drv_get_time_us(const struct uwb_chip *uwb);

/**
 * llhw_drv_resync_dtu_systime() - Resync DTU time and SysTime.
 * @uwb: the UWB device.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_drv_resync_dtu_systime(struct uwb_chip *uwb);

/**
 * llhw_drv_idle() - Start a timer to wake-up from idle state.
 * @idle_duration_us: timer delay is microseconds.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_drv_idle(uint32_t idle_duration_us);

/**
 * llhw_drv_set_deep_sleep() - Put UWB transceiver into deep-sleep mode.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_drv_set_deep_sleep(void);

/**
 * llhw_drv_wakeup() - Wake-up UWB transceiver from deep-sleep mode.
 * @uwb: the UWB device.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_drv_wakeup(struct uwb_chip *uwb);

/**
 * llhw_drv_hrp_params_to_dwt() - Convert HRP UWB params to DWT params..
 * @params: the HRP UWB params.
 * @psr: converted PSR value.
 * @sfd_type: converted SFD Type value.
 * @sfd_timeout: converted SFD Timeout value.
 * @data_rate: converted Data Rate value.
 * @phr_rate: converted PHR Rate value.
 * @phr_mode: converted PHR mode value.
 * @rx_pac: converted RX PAC value.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_drv_hrp_params_to_dwt(const struct mcps802154_hrp_uwb_params *params,
			       int *psr, dwt_sfd_type_e *sfd_type,
			       uint16_t *sfd_timeout,
			       dwt_uwb_bit_rate_e *data_rate,
			       dwt_phr_rate_e *phr_rate,
			       dwt_phr_mode_e *phr_mode,
			       dwt_pac_size_e *rx_pac);

/**
 * llhw_drv_set_hrp_params() - Set HRP UWB params.
 * @uwb: the UWB device.
 * @params: the HRP MCPS params.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_drv_set_hrp_params(struct uwb_chip *uwb,
			    const struct mcps802154_hrp_uwb_params *params);

/**
 * llhw_drv_get_rssi_constant() - Return RSSI constant specific to the chip.
 *
 * Return: RSSI constant.
 */
float llhw_drv_get_rssi_constant(void);

/**
 * llhw_drv_adjust_tx_power() - Adjust TX Power.
 * @offset: TX power offset to apply in 0.1dB units.
 * @ref_tx_power: Reference TX power on top of which the boost shall be applied.
 * @channel: UWB channel.
 * @adj_tx_power: returned adjusted TX power.
 * @app_offset: returned applied TX power offset.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_drv_adjust_tx_power(int16_t offset, uint32_t ref_tx_power,
			     uint8_t channel, uint32_t *adj_tx_power,
			     int16_t *app_offset);

/**
 * llhw_drv_cb_data_to_uwb() - Convert callback data to `struct uwb_chip`.
 * @cb_data: callback data.
 *
 * Return: structure uwb_chip representing the UWB device..
 */
struct uwb_chip *llhw_drv_cb_data_to_uwb(const dwt_cb_data_t *cb_data);

#endif /* __LLHW_DRV__H__ */
