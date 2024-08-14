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

#ifndef _DECA_COMPAT_EXTENDED_H_
#define _DECA_COMPAT_EXTENDED_H_

#include "deca_interface.h"
#include "deca_device_api.h"

/**
 * dwt_update_dw_extended - Update local dw pointer.
 * @new_dw - dw instantiated by MCPS layer.
 */
void dwt_update_dw_extended(struct dwchip_s *new_dw);

/**
 * dwt_setstslength - Set the STS length.
 * @stsblocks - number of STS 8us blocks (0 == 8us, 1 == 16us, etc).
 */
void dwt_setstslength(uint8_t stsblocks);

/**
 * dwt_getframelength_ext - Read the frame length of the last received frame.
 * @ranging_bit: ranging bit (unused).
 *
 * Return: frame length.
 */
uint16_t dwt_getframelength_ext(uint8_t *ranging_bit);

/**
 * dwt_readstsquality_ext - Read the STS Quality Index.
 * @rxStsQualityIndex: returned STS quality index.
 * @stsSegment: STS segment (unused).
 *
 * Return: positive or zero on success, else a negative error code.
 */
int dwt_readstsquality_ext(int16_t *rxStsQualityIndex, int stsSegment);

#endif /* _DECA_COMPAT_EXTENDED_H_ */
