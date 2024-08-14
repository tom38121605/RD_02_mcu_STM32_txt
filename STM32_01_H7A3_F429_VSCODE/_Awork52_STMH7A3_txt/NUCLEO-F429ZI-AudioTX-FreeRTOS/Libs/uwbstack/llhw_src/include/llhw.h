/*
 * Copyright (c) 2021 Qorvo, Inc
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
#ifndef __LLHW_H__
#define __LLHW_H__

#include <stdint.h>
#include <linux/atomic.h>

#include "net/mcps802154.h"

#include "llhw_config.h"
#include "llhw_config_and_calib.h"
#include "llhw_drv_config.h"

#define MAX_MSG_LEN 4095

#ifdef UWBMAC_BUF_MAX_SIZE
#define MAX_RX_LEN UWBMAC_BUF_MAX_SIZE
#else
#define MAX_RX_LEN MAX_MSG_LEN
#endif

#define UWB_TO_DWTCONFIG(uwb) (&uwb->dwt_config)
#define UWB_TO_RUNTIME(uwb) (&uwb->runtime)
#define UWB_TO_DEEP_SLEEP_STATE(uwb) (&uwb->runtime.deep_sleep_state)

/**
 * enum uwb_rx_flags - Additional RX information flags.
 * @UWB_RX_FLAG_AACK: set if an automatic ack is sent.
 * @UWB_RX_FLAG_ND: set if no data.
 * @UWB_RX_FLAG_RSSI: set if RSSI data is available.
 */
enum uwb_rx_flags {
	UWB_RX_FLAG_AACK = 1 << 0,
	UWB_RX_FLAG_ND = 1 << 1,
	UWB_RX_FLAG_RSSI = 1 << 2,
};

/**
 * enum uwb_operational_state - UWB transceiver Power Operational State.
 * @UWB_OP_STATE_OFF: completely off.
 * @UWB_OP_STATE_DEEP_SLEEP: in deep-sleep state.
 * @UWB_OP_STATE_SLEEP: in sleep-state.
 * @UWB_OP_STATE_WAKE_UP: wake-up in progress.
 * @UWB_OP_STATE_INIT_RC: waked-up and in INIT_RC state.
 * @UWB_OP_STATE_IDLE_RC: waked-up and in IDLE_RC state.
 * @UWB_OP_STATE_IDLE_PLL: waked-up and in IDLE_PLL state.
 * @UWB_OP_STATE_TX: must perform a TX after wake-up.
 * @UWB_OP_STATE_RX: must perform a RX after wake-up.
 * @UWB_OP_STATE_MAX: max number of Power Operational States.
 */
enum uwb_operational_state {
	UWB_OP_STATE_OFF = 0,
	UWB_OP_STATE_DEEP_SLEEP,
	UWB_OP_STATE_SLEEP,
	UWB_OP_STATE_WAKE_UP,
	UWB_OP_STATE_INIT_RC,
	UWB_OP_STATE_IDLE_RC,
	UWB_OP_STATE_IDLE_PLL,
	UWB_OP_STATE_TX,
	UWB_OP_STATE_RX,
	UWB_OP_STATE_MAX,
};

/** Callback to call at wake-up from idle or deep-sleep. */
typedef int (*uwb_wakeup_done_cb)(struct mcps802154_llhw *llhw);

/**
 * struct uwb_deep_sleep_state - Useful data to restore on wake-up.
 * @next_operational_state: operational state to enter after DEEP SLEEP mode.
 * @config_changed: bitfield of configuration changed during DEEP-SLEEP.
 * @frame_idx: index of the frame in the round.
 * @start_wifi_coex_at_wakeup: indicate if wifi coex must be started at wake-up.
 * @filt: Hardware address filtering settings.
 * @tx_skb: saved frame to transmit for deferred TX.
 * @tx_config: saved info to use for deferred TX.
 * @rx_config: saved parameter for deferred RX.
 * @wakeup_done_cb: callback to call at wake-up.
 */
struct uwb_deep_sleep_state {
	enum uwb_operational_state next_operational_state;
	unsigned long config_changed;
	int frame_idx;
	bool start_wifi_coex_at_wakeup;
	struct ieee802154_hw_addr_filt filt;
	struct sk_buff *tx_skb;
	union {
		struct mcps802154_tx_frame_config tx_config;
		struct mcps802154_rx_frame_config rx_config;
	};
	uwb_wakeup_done_cb wakeup_done_cb;
};

/** enum uwb_rctu_conv_state - DTU to RCTU conversion state
 * @UNALIGNED: need to redo all
 * @ALIGNED: aligned to DTU but not synced yet with RCTU
 * @ALIGNED_SYNCED: all done
 */
enum uwb_rctu_conv_state { UNALIGNED = 0, ALIGNED, ALIGNED_SYNCED };

/**
 * struct uwb_rctu_conv - DTU to RCTU conversion data
 * @state: current state of converter
 * @alignment_rmarker_dtu:  alignment DTU value
 * @synced_rmarker_rctu: rmarker RCTU value
 */
struct uwb_rctu_conv {
	enum uwb_rctu_conv_state state;
	u32 alignment_rmarker_dtu;
	u64 synced_rmarker_rctu;
};

/**
 * struct uwb_rssi_sample - RSSI sample
 * @accum: number of accumulated symbols for the CIR.
 * @power: estimation of channel power for the CIR sequence.
 */
struct uwb_rssi_sample {
	uint32_t accum;
	uint32_t power;
};

/**
 * struct uwb_rssi_samples - RSSI samples.
 * @samples: table of RSSI samples.
 */
struct uwb_rssi_samples {
	struct uwb_rssi_sample samples[CIA_RX_NUM][CIA_CIR_NUM];
};

/**
 * struct uwb_rx - UWB RX frame informations.
 * @flags: RX information flags.
 * @skb: buffer where received frame data is stored.
 * @cfo: clock-offset of the received frame.
 * @rssi_samples: RSSI samples if the received frame.
 */
struct uwb_rx {
	uint32_t flags;
	struct sk_buff *skb;
	int16_t cfo;
	struct uwb_rssi_samples rssi_samples;
};

/**
 * enum uwb_loopback_state - Loopback test machine states.
 * @UWB_MCPS_LOOPBACK_STATE_STARTED: loopback test has been started.
 * @UWB_MCPS_LOOPBACK_STATE_TX_DONE: TX frame has been sent.
 * @UWB_MCPS_LOOPBACK_STATE_RX_OK: RX frame has been received successfully.
 * @UWB_MCPS_LOOPBACK_STATE_RX_ERR: RX frame timed-out or has been received with
 * errors.
 */
enum uwb_loopback_state {
	UWB_MCPS_LOOPBACK_STATE_STARTED = 0,
	UWB_MCPS_LOOPBACK_STATE_TX_DONE,
	UWB_MCPS_LOOPBACK_STATE_RX_OK,
	UWB_MCPS_LOOPBACK_STATE_RX_ERR,
};

/**
 * enum uwb_trx_state - Transmission/reception state.
 * @TRX_IDLE: IDLE state, no TX nor RX.
 * @TRX_TX_ARMED: TX is armed.
 * @TRX_RX_ARMED: RX is armed.
 * @TRX_RX_NOTIF: RX event occured and RX idle.
 */
enum uwb_trx_state {
	TRX_IDLE,
	TRX_TX_ARMED,
	TRX_RX_ARMED,
	TRX_RX_NOTIF,
};

/**
 * struct uwb_trx_date - tx or rx date data.
 * @date_dtu: TX or RX date in dtu.
 * @armed: True if date_dtu has to be considered as valid.
 */
struct uwb_trx_date {
	uint32_t date_dtu;
	enum uwb_trx_state state;
};

/**
 * struct uwb_runtime - Runtime variables.
 * @current_operational_state: current power operational state.
 * @rctu_conv: DTU to RCTU conversion informations.
 * @time_zero_us: initial time in us to convert time to/from DTU.
 * @dtu_sync: last synchro DTU.
 * @systime_sync: last synchro Systime.
 * @deep_sleep_state: informations needed for deep-sleep management.
 * @need_ranging_clock: true if next operation needs ranging clock.
 * @frame_skb: buffer used for read frames.
 * @loopback_state: loopback state.
 * @uwb_to_wlan_active: current state of UWB to WLAN wifi coex gpio.
 * @wlan_to_uwb_active: current state of WLAN to UWB wifi coex gpio.
 * @auto_ack_enabled: current status of auto-ack.
 * @trx_date: TX/RX timestamp structure to handle disabling of the TX or RX.
 */
struct uwb_runtime {
	enum uwb_operational_state current_operational_state;
	struct uwb_rctu_conv rctu_conv;
	int64_t time_zero_us;
	uint32_t dtu_sync;
	uint32_t systime_sync;
	struct uwb_deep_sleep_state deep_sleep_state;
	bool need_ranging_clock;
	struct sk_buff *frame_skb;
	enum uwb_loopback_state loopback_state;
	bool uwb_to_wlan_active;
	bool wlan_to_uwb_active;
	int auto_ack_enabled;
	struct uwb_trx_date trx_date;
};

/**
 * enum uwb_driver_callbacks - Callbacks used in UWB driver.
 * @cbTxDone: called when chip has received IRQ for TX done.
 * @cbRxOk: called when chip has received IRQ for successful RX.
 * @cbRxTo: called when chip has received IRQ for timed-out RX.
 * @cbRxErr: called when chip has received IRQ for RX with error.
 * @cbDevErr: called when chip has received IRQ for a device error.
 * @cbSPIRdy: called when chip has received IRQ for SPI ready.
 * @cbSysEvent: called when chip has received IRQ for System Event or Dual SPI.
 * @cbTimerExpired: called when chip has received IRQ for timer expiration.
 */
struct uwb_driver_callbacks {
	dwt_cb_t cbTxDone;
	dwt_cb_t cbRxOk;
	dwt_cb_t cbRxTo;
	dwt_cb_t cbRxErr;
	dwt_cb_t cbDevErr;
	dwt_cb_t cbSPIRdy;
	dwt_cb_t cbSysEvent;
	dwt_cb_t cbTimerExpired;
};

/**
 * struct uwb_interrupts_config - Interrupts configuration.
 * @bitmask_lo: LSB interrupt bitmask.
 * @bitmask_hi: MSB interrupt bitmask.
 * @options: interrupt working options.
 */
struct uwb_interrupts_config {
	uint32_t bitmask_lo;
	uint32_t bitmask_hi;
	dwt_INT_options_e options;
};

/**
 * struct uwb_rffe_config - RFFE configuration.
 * @mode: RFFE mode (off, fixed or lut).
 * @rfpath1: combination of flags to define first RF path.
 * @rfpath2: combination of flags to define second RF path.
 */
struct uwb_rffe_config {
	uint8_t mode;
	uint16_t rfpath1;
	uint16_t rfpath2;
};

/**
 * struct uwb_chip - The UWB device, its configuration and current state.
 * @dwt_config: driver configuration.
 * @hrp_params: HRP parameters.
 * @runtime: runtime variables.
 * @rx: informations about received frame.
 * @config_and_calib: configuration and Calibration informations.
 * @cbs: driver callbacks.
 * @interrupt_config: IRQ configuration.
 * @ant_cal_info: Antenna calibration parameters.
 * @default_rfsw_cfg: default RF Switch configuration.
 * @smart_tx_power: smart TX power applied.
 * @rx_config_drv: RX receiver configuration.
 * @sts_key: STS key.
 * @sts_iv: STS IV.
 * @seg_len: STS segment length.
 * @current_rffe_cfg: current RFFE configuration.
 * @short_addr: short address.
 * @pan_id: pan ID.
 * @frame_filter: frame filter.
 * @frame_filter_mode: frame filter mode.
 * @xtal_trim: tuned crystal trim value.
 * @ant_set_id: antenna set ID.
 * @enable_hw_assistance: Enable the HW filtering and the TX/RX FCS handling.
 * @rx_updated: indicates that RX parameters have changed since last config.
 * @tx_updated: indicates that TX parameters have changed since last config.
 * @chips_per_pac: chips per PAC unit.
 * @pre_timeout_pac: preamble timeout in PAC unit.
 * @pdoa_offset: PDOA offset for axis X, Y and Z.
 * @pdoa_lut: PDOA lut for axis X, Y and Z.
 * @iface_is_started: interface status, true when mcps start() has been called.
 * @opportunistic_deep_sleep: true when chip supports opportunistic deep-sleep.
 * @wifi_coex_hw_assisted: true when gpio coex pin can be automatically driven
 * high by the hardware.
 * @wifi_coex_delay_us: WiFi coex delay in microseconds.
 * @wifi_coex_mode: WiFi coex mode.
 * @rf_noise_offset: RF noise offset (dB) to apply when calculating the RX SNR.
 * @idle_timer_running: True if Idle timer is started and not expired yet.
 * @priv: LLHW driver private data.
 */
struct uwb_chip {
	dwt_config_t dwt_config;
	struct mcps802154_hrp_uwb_params hrp_params;
	struct uwb_runtime runtime;
	struct uwb_rx *rx;
	struct llhw_config_and_calib *config_and_calib;
	struct uwb_driver_callbacks cbs;
	struct uwb_interrupts_config interrupt_config;
	struct antenna_calib_prf ant_cal_info;
	void *default_rfsw_cfg;
	bool smart_tx_power;
	uint32_t rx_config_drv;
	dwt_sts_cp_key_t sts_key;
	dwt_sts_cp_iv_t sts_iv;
	int seg_len;
	struct uwb_rffe_config current_rffe_cfg;
	uint16_t short_addr;
	uint16_t pan_id;
	uint16_t frame_filter;
	uint16_t frame_filter_mode;
	uint8_t xtal_trim;
	int8_t ant_set_id;
	bool enable_hw_assistance;
	bool rx_updated;
	bool tx_updated;
	int chips_per_pac;
	int pre_timeout_pac;
	int16_t pdoa_offset[AOA_TYPE_AXIS_MAX];
	const pdoa_lut_t *pdoa_lut[AOA_TYPE_AXIS_MAX];
	atomic_t iface_is_started;
	bool opportunistic_deep_sleep;
	bool wifi_coex_hw_assisted;
	uint32_t wifi_coex_delay_us;
	enum uwb_wifi_coex_mode wifi_coex_mode;
	int8_t rf_noise_offset;
	bool idle_timer_running;
	void *priv;
};

/**
 * llhw_init() - Init Low-Level Hardware layer.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_init(void);

/**
 * llhw_do_set_hw_addr_filt() - set hardware filter parameters if changed.
 * @uwb: the UWB device
 * @filt: the hardware filter parameters
 * @ptr_changed: bitfield indicating which parameter has changed
 *
 * Return: zero on success, else a negative error code.
 */
void llhw_do_set_hw_addr_filt(struct uwb_chip *uwb,
			      struct ieee802154_hw_addr_filt *filt,
			      unsigned long *ptr_changed);

/**
 * llhw_wakeup_from_timer() - Perform needed actions after wake-up timer
 * expiration.
 * @dss: Deep-sleep State informations.
 *
 * Return: zero on success, else a negative error code.
 */
void llhw_wakeup_from_timer(void);

/**
 * llhw_set_deep_sleep() - Put UWB transceiver into deep-sleep mode.
 * @uwb: the UWB device.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_set_deep_sleep(struct uwb_chip *uwb);

/**
 * llhw_wakeup_done() - Perform needed actions after wake-up from deep-sleep.
 * @uwb: the UWB device.
 *
 * Return: zero on success, else a negative error code.
 */
int llhw_wakeup_done(struct uwb_chip *uwb);

/**
 * llhw_coex_start() - start Wifi Coex at the expected date: either set the GPIO
 * right now, or start a timer to set it later.
 * @uwb: the UWB device.
 * @trx_delayed: indicate if the next TX/RX is delayed or immediate.
 * @trx_date_dtu: DTU date of the next TX/RX (in case trx_delayed is 1).
 * @cur_time_dtu: Current DTU date.
 * @frame_idx: index of the frame in the round.
 *
 * Return: true is timer has been started (wifi coex delayed), false otherwise.
 */
bool llhw_coex_start(struct uwb_chip *uwb, int *trx_delayed,
		     uint32_t *trx_date_dtu, uint32_t cur_time_dtu,
		     int frame_idx);

/**
 * llhw_coex_set() - set Wifi Coex GPIO.
 * @uwb: the UWB device
 */
void llhw_coex_set(struct uwb_chip *uwb);

/**
 * llhw_coex_stop() - reset Wifi Coex GPIO.
 * @uwb: the UWB device.
 */
void llhw_coex_stop(struct uwb_chip *uwb);

/**
 * llhw_get_pdoa_fom() - compute the figure of merit for PDoA.
 * @sts_fom: the STS FoM
 *
 * If the STS FoM is less than sts_fom_threshold, PDoA FoM is 1, the worst.
 * If the STS FoM is greater or equal than sts_fom_threshold,
 * sts_fom_threshold to 255 values are mapped to 2 to 255.
 * A FoM of 0 means an invalid measurement.
 *
 * Return: the PDoA FoM.
 */
uint8_t llhw_get_pdoa_fom(const uint8_t sts_fom);

/**
 * llhw_get_sts_fom() - compute the figure of merit of STS
 * @uwb: pointer to the uwb chip structure.
 * @sts_segment: the STS segment to consider [0;3]
 *
 * Return: the STS FoM
 */
uint8_t llhw_get_sts_fom(const struct uwb_chip *uwb, const uint8_t sts_segment);

/**
 * llhw_read_frame_info() - Read the last received frame.
 * @uwb: the UWB device.
 * @data_length: data length, or -1 if unknown.
 *
 * This function reads the last received frame.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_read_frame_info(struct uwb_chip *uwb, int data_length);

/**
 * llhw_check_hrp_uwb_params() - Check that the HRP UWB params are supported.
 * @llhw: the Low-level hardware without MCPS.
 * @params: HRP UWB parameters.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_check_hrp_uwb_params(struct mcps802154_llhw *llhw,
			      const struct mcps802154_hrp_uwb_params *params);

/**
 * llhw_set_trx_date() - set the TRX date structure.
 * @uwb: the UWB device.
 * @state: State to set.
 * @date_dtu: Date of next access.
 */
void llhw_set_trx_date(struct uwb_chip *uwb, enum uwb_trx_state state,
		       u32 date_dtu);

/**
 * llhw_seg_len_to_dwt() - Convert segment length to enum `dwt_sts_lengths_e`.
 * @uwb: the UWB chip.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_seg_len_to_dwt(struct uwb_chip *uwb);

/**
 * llhw_idle_expired() - Process Idle timer expiration.
 * @uwb: the UWB device.
 */
void llhw_idle_expired(struct uwb_chip *uwb);

/**
 * llhw_update_timings() - Update configuration dependent timings.
 * @llhw: the Low-level hardware without MCPS.
 */
void llhw_update_timings(struct mcps802154_llhw *llhw);

/**
 * llhw_get_prf_calib_index() - Convert MCPS PRF value to calib PRF index.
 * @prf: MCPS PRF value.
 *
 * Return: Calib PRF index value.
 */
int llhw_get_prf_calib_index(enum mcps802154_prf prf);

/**
 * llhw_update_antenna_channel_config() - Update configuration depending on
 * antenna and channel used.
 * @uwb: the UWB device.
 *
 * Return: 0 on success, else a negative error code.
 */
int llhw_update_antenna_channel_config(struct uwb_chip *uwb);

#endif /* __LLHW_H__ */
