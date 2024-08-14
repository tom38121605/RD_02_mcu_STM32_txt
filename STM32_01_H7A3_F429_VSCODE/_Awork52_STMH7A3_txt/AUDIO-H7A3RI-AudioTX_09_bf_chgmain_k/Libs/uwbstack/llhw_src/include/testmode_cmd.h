/*
 * Copyright (c) 2020-2021 Qorvo, Inc
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

#ifndef TESTMODE_CMD
#define TESTMODE_CMD

/**
 * enum testmode_cmd_id - Test mode command identifiers.
 * @TESTMODE_CMD_ID_MIN: Minimum value, RESERVED.
 * @TESTMODE_CMD_ID_TX_CONT_WAVE: Transmit Continous Wave.
 * @TESTMODE_CMD_ID_PLL_LOCK: PLL lock test.
 * @TESTMODE_CMD_ID_MAX: Maximum value, RESERVED.
 */
enum testmode_cmd_id {
	TESTMODE_CMD_ID_MIN = 0,
	TESTMODE_CMD_ID_TX_CONT_WAVE,
	TESTMODE_CMD_ID_PLL_LOCK,
	TESTMODE_CMD_ID_MAX = 255
};

/**
 * struct testmode_params_tx_cont_wave - Params for TX continuous wave test.
 * @enable: Boolean to enable or disable test mode.
 * @ant_set_id: Antenna set index to use for transmit.
 * @channel: Channel to use for test.
 */
struct testmode_params_tx_cont_wave {
	int ant_set_id;
	bool enable;
	uint8_t channel;
};

/**
 * enum testmode_pll_lock_errcode - Errcode returned in PLL lock test.
 * @TESTMODE_PLL_LOCK_SUCCESS: Test success.
 * @TESTMODE_PLL_LOCK_ERROR: PLL Lock error.
 * @TESTMODE_PLL_LOCK_INTERNAL_ERROR: Internal error.
 */
enum testmode_pll_lock_errcode {
	TESTMODE_PLL_LOCK_SUCCESS,
	TESTMODE_PLL_LOCK_ERROR,
	TESTMODE_PLL_LOCK_INTERNAL_ERROR,
};

/**
 * struct testmode_params_pll_lock - Params for PLL lock test.
 * @pll_status: Returned PLL status.
 * @channel: Input channel to use for test.
 * @errcode: Returned error code from `enum testmode_pll_lock_errcode`.
 */
struct testmode_params_pll_lock {
	uint32_t pll_status;
	uint8_t channel;
	uint8_t errcode;
};

/**
 * struct testmode_info - Test mode information.
 * @cmd_id: Command ID.
 * @tx_cont_wave: Parameters specific to command TESTMODE_CMD_ID_TX_CONT_WAVE.
 * @pll_lock: Parameters specific to command TESTMODE_CMD_ID_PLL_LOCK.
 */
struct testmode_info {
	enum testmode_cmd_id cmd_id;
	union {
		struct testmode_params_tx_cont_wave tx_cont_wave;
		struct testmode_params_pll_lock pll_lock;
	};
};
#endif /* TESTMODE_CMD */
