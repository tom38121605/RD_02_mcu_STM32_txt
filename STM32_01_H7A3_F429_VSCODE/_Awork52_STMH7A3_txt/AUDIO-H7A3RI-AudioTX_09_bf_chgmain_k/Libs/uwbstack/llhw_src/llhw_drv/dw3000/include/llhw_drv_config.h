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

#ifndef _UWB_DRIVER_CONFIG_H_
#define _UWB_DRIVER_CONFIG_H_

#include <linux/types.h>

#include "deca_interface.h"
#include "deca_device_api.h"

#include "deca_compat_extended.h"

#define CALIBRATION_PDOA_LUT_MAX 31

#define CIA_RX_NUM 2
#define CIA_CIR_NUM 5

#define DW3000_DT_SPI_NODE DT_ALIAS(dw35720_spi)

#define DW3000_DT_DEV_NODE dw35720_dev
#define DW3000_DT_LBL DT_CHILD(DW3000_DT_SPI_NODE, DW3000_DT_DEV_NODE)
#define DW3000_DT_RSTN_DEV DT_PHANDLE(DW3000_DT_LBL, rstn_gpios)
#define DW3000_DT_RSTN_PIN DT_PHA_BY_IDX(DW3000_DT_LBL, rstn_gpios, 0, pin)
#define DW3000_DT_RSTN_FLAGS DT_PHA_BY_IDX(DW3000_DT_LBL, rstn_gpios, 0, flags)
#define DW3000_DT_IRQ_GPIO_DEV DT_PHANDLE(DW3000_DT_LBL, irq_gpios)
#define DW3000_DT_IRQ_PIN DT_PHA_BY_IDX(DW3000_DT_LBL, irq_gpios, 0, pin)
#define DW3000_DT_IRQ_FLAGS DT_PHA_BY_IDX(DW3000_DT_LBL, irq_gpios, 0, flags)
#define DW3000_DT_WUP_EXISTS DT_NODE_HAS_PROP(DW3000_DT_LBL, wup_gpios)
#define DW3000_DT_WUP_DEV DT_PHANDLE(DW3000_DT_LBL, wup_gpios)
#define DW3000_DT_WUP_PIN DT_PHA_BY_IDX(DW3000_DT_LBL, wup_gpios, 0, pin)
#define DW3000_DT_WUP_FLAGS DT_PHA_BY_IDX(DW3000_DT_LBL, wup_gpios, 0, flags)

#define GET_GPIO_DEV() device_get_binding(DT_LABEL(DW3000_DT_IRQ_GPIO_DEV))

/* Chip capabilities. */
#define UWB_LLHW_PRF_MODE_SUPPORTED (MCPS802154_LLHW_BPRF)
#define UWB_LLHW_DATA_RATE_SUPPORTED \
	(MCPS802154_LLHW_DATA_RATE_850K | MCPS802154_LLHW_DATA_RATE_6M81)
#define UWB_LLHW_PRF_SUPPORTED (MCPS802154_LLHW_PRF_16 | MCPS802154_LLHW_PRF_64)
#define UWB_LLHW_PHR_DATA_RATE_SUPPORTED      \
	(MCPS802154_LLHW_PHR_DATA_RATE_850K | \
	 MCPS802154_LLHW_PHR_DATA_RATE_6M81)
#define UWB_LLHW_PSR_SUPPORTED                               \
	(MCPS802154_LLHW_PSR_32 | MCPS802154_LLHW_PSR_64 |   \
	 MCPS802154_LLHW_PSR_128 | MCPS802154_LLHW_PSR_256 | \
	 MCPS802154_LLHW_PSR_1024 | MCPS802154_LLHW_PSR_4096)
#define UWB_LLHW_SFD_SUPPORTED \
	(MCPS802154_LLHW_SFD_4A | MCPS802154_LLHW_SFD_4Z_8)
#define UWB_LLHW_STS_SEG_SUPPORTED (MCPS802154_LLHW_STS_SEGMENT_1)
#define UWB_LLHW_AOA_SUPPORTED                                         \
	(MCPS802154_LLHW_AOA_AZIMUTH | MCPS802154_LLHW_AOA_ELEVATION | \
	 MCPS802154_LLHW_AOA_FOM)

#endif /* _UWB_DRIVER_CONFIG_H_ */
