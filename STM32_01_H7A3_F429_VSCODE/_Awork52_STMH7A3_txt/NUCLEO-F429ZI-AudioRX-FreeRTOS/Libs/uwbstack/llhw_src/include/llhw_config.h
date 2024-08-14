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

#ifndef __LLHW_CONFIG_H__
#define __LLHW_CONFIG_H__

#ifndef M_PI
#define M_PI (3.141592654f)
#endif

#if defined(CONFIG_DWT_UWB)
#define UWB_SPIRDY_BIT_MASK DWT_INT_SPIRDY_BIT_MASK
#else
#define UWB_SPIRDY_BIT_MASK 0
#endif

/* UWB config */
/* clang-format off */
#define DEFAULT_CHANNEL			9
#define DEFAULT_TXPREAMBLENGTH 		MCPS802154_PSR_64
#define DEFAULT_PRF			MCPS802154_PRF_64
#define DEFAULT_PCODE 			9
#define DEFAULT_SFD 			MCPS802154_SFD_4Z_8
#define DEFAULT_DATARATE 		MCPS802154_DATA_RATE_6M81
#define DEFAULT_PSDUSIZE 		MCPS802154_HRP_UWB_PSDU_SIZE_1023
#define DEFAULT_PHRHIGHRATE 		false
#define DEFAULT_STS_MODE 		DWT_STS_MODE_OFF /* (DWT_STS_MODE_1 | DWT_STS_MODE_SDC) */
#define DEFAULT_STS_LENGTH 		64
#ifdef CONFIG_DWT_UWB
#define DEFAULT_PDOA_MODE 		DWT_PDOA_M1
#else
#define DEFAULT_PDOA_MODE 		DWT_PDOA_M0
#endif


#define DEFAULT_FF 				DWT_FF_ENABLE_802_15_4
#define DEFAULT_FF_MODE 		(DWT_FF_BEACON_EN | DWT_FF_DATA_EN \
								| DWT_FF_ACK_EN | DWT_FF_MAC_EN \
								| DWT_FF_MULTI_EN)
#define DEFAULT_DO_TX_CONFIG 		true
#define DEFAULT_PGDLY 			0x34
#define DEFAULT_POWER 			0xfdfdfdfdUL
#define DEFAULT_PGCOUNT 		0
#define DEFAULT_LED_MODE 		(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK)
#ifdef CONFIG_DWT_UWB
#define DEFAULT_RFFE_MODE		DWT_LNA_PA_DISABLE
#define DEFAULT_RFFE_RFPATH1		0 /* unused for DW3000 */
#define DEFAULT_RFFE_RFPATH2		0 /* unused for DW3000 */
#define DEFAULT_PA_LNA_RXB 		(0)
#define DEFAULT_LNA_RXA 		(0)
#else
/* Default mode must be synchronized with default states */
#define DEFAULT_RFFE_MODE		DWT_RFFE_LUT_MODE
#define DEFAULT_RFFE_RFPATH1		DWT_RFPATH_LNA
#define DEFAULT_RFFE_RFPATH2		0
#endif
#define DEFAULT_INT_CONFIG_BITMASK_LO 	(UWB_SPIRDY_BIT_MASK    | DWT_INT_TXFRS_BIT_MASK |  \
					 DWT_INT_RXFCG_BIT_MASK | DWT_INT_ARFE_BIT_MASK  | \
					 DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | \
					 DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | \
					 DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | \
					 DWT_INT_TIMER0_BIT_MASK | DWT_INT_TIMER1_BIT_MASK)
#define DEFAULT_INT_CONFIG_BITMASK_HI	0
#define DEFAULT_INT_CONFIG_OPTIONS 	DWT_ENABLE_INT_ONLY
#define DEFAULT_DO_STS_KEYIV_CONFIG 	false
#define DEFAULT_CIA_DIAG_LOG		DW_CIA_DIAG_LOG_ALL
#ifdef CONFIG_DWT_UWB
#define DEFAULT_RX_CONFIG 		0
#else
#define DEFAULT_RX_CONFIG 		(CHAN_CTRL_RX_A_ON | CHAN_CTRL_RX_B_ON)
#endif

/* RF Noise Figure (dB).
 * DW3x NF at sensitivity is typically -6.94 dB (CH5) or -7.3 dB (CH9). */
#define DEFAULT_RF_NOISE_OFFSET		(-7)

/* run-time config */
#define DEFAULT_AUTOSTART_NONE		(0)
#define DEFAULT_NODE_ADDR		0x0001 /**< Addr16    */
#define DEFAULT_PANID			0xDECA /**< PanID */
#define DEFAULT_UART			1 /**< output to the UART */
#define DEFAULT_ANTD_BASE 		(513.484f * 1e-9 / DWT_TIME_UNITS) /*Total antenna delay*/
#define DEFAULT_ANTD 			(uint16_t)(0.5 * DEFAULT_ANTD_BASE)
#define DEFAULT_PDOAOFF_DEG 		0 /**< Phase Differences offset */
#define DEFAULT_RNGOFF 			0 /**< Ranging offset */
#define DEFAULT_PDOA_TEMP 		0 /**< Linear temperature coefficient for PDOA, mrad. Diff of PDOA when temperature changed +20C deg centigrade */
#define DEFAULT_PHASECORR_EN 		0 /**< enable antenna related phase correction polynomial */
#define DEFAULT_REPORT_LEVEL 		1 /**< what to use as output of TWR: 1:JSON, 2:Reduced, 3:Minimal */
#define DEFAULT_DEBUG 			0 /**< if 1, then the LED_RED used to show an error, if any */

#define DTU_PER_SYS_POWER	4

#define CHIP_FREQ		499200000
#define CHIP_PER_SYS 		2
#define CHIP_PER_DTU 		(CHIP_PER_SYS * (1 << DTU_PER_SYS_POWER))

#define CHIP_PER_SYMB_HPRF 364
#define CHIP_PER_SYMB_PRF64 508
#define CHIP_PER_SYMB_PRF16 496

#define CHIP_PER_DLY		512
#define RCTU_PER_CHIP		128
#define RCTU_PER_DTU 		(RCTU_PER_CHIP * CHIP_PER_DTU)
#define RCTU_PER_SYS 		(RCTU_PER_CHIP * CHIP_PER_SYS)
#define RCTU_PER_DLY 		(CHIP_PER_DLY / RCTU_PER_CHIP)

#define DTU_FREQ		(CHIP_FREQ / CHIP_PER_DTU)

/* 6.9.1.5 in 4z, for HRP UWB PHY:
   416 chips = 416 / (499.2 * 10^6) ~= 833.33 ns */
#define DTU_PER_RSTU		(416 / CHIP_PER_DTU)
#define DTU_PER_DLY		(CHIP_PER_DLY / CHIP_PER_DTU)
#define SYS_PER_DLY		(CHIP_PER_DLY / CHIP_PER_SYS)

#define ANTICIP_DTU		(16 * (DTU_FREQ / 1000))

#if defined(CONFIG_DWT_UWB)
#define RX_ENABLE_STARTUP_DLY 16
#else
#define RX_ENABLE_STARTUP_DLY 20
#endif
#define RX_ENABLE_STARTUP_DTU    (RX_ENABLE_STARTUP_DLY * CHIP_PER_DLY / \
					 CHIP_PER_DTU)

#define PREAMBLE_INDEX_HPRF_START 25
#define PREAMBLE_INDEX_PRF64_START 9

/* Default number of taps contained in the CIR. */
#define CIR_DEFAULT_NB_TAPS_IN_CIR (16)
/* Default precursor offset of the first path in the CIR. */
#define CIR_DEFAULT_PRECURSOR_OFFSET (-8)
/* Size of tap in CIR. 4 bytes for real part + 4 bytes for imaginary part
NOTE This value might be different for DW3000. */
#define CIR_TAP_SIZE (8)

/* clang-format on */

#define PDOADEG2PDOA(pdoa_deg)                                               \
	({                                                                   \
		uint16_t off;                                                \
		if (pdoa_deg > 0)                                            \
			off = (int)(-1 * (pdoa_deg)*M_PI) * (1 << 11) / 180; \
		else                                                         \
			off = (int)((360 - pdoa_deg) * M_PI) * (1 << 11) /   \
			      180;                                           \
		off;                                                         \
	})

#endif /* __LLHW_CONFIG_H__ */
