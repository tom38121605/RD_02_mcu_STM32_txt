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

#ifndef _LLHW_DRV_IRQ_WRAPPER_H_
#define _LLHW_DRV_IRQ_WRAPPER_H_

#include <stdint.h>

void llhw_drv_irq_wrap_connect(void);
void llhw_drv_irq_wrap_enable(void);
void llhw_drv_irq_wrap_disable(void);
uint32_t llhw_drv_irq_wrap_check(void);

#endif /* _LLHW_DRV_IRQ_WRAPPER_H_ */
