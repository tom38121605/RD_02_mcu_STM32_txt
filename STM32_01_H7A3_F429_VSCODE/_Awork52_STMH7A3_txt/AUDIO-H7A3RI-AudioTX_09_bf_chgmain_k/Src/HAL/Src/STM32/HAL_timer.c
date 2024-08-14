/**
 * @file    HAL_timer.c
 *
 * @brief   HAL functions for timer
 *
 * @author  Decawave Applications
 *
 * @attention Copyright (c) 2021 - 2022, Qorvo US, Inc.
 * All rights reserved
 * Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions, and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 3. You may only use this software, with or without any modification, with an
 *  integrated circuit developed by Qorvo US, Inc. or any of its affiliates
 *  (collectively, "Qorvo"), or any module that contains such integrated circuit.
 * 4. You may not reverse engineer, disassemble, decompile, decode, adapt, or
 *  otherwise attempt to derive or gain access to the source code to any software
 *  distributed under this license in binary or object code form, in whole or in
 *  part.
 * 5. You may not use any Qorvo name, trademarks, service marks, trade dress,
 *  logos, trade names, or other symbols or insignia identifying the source of
 *  Qorvo's products or services, or the names of any of Qorvo's developers to
 *  endorse or promote products derived from this software without specific prior
 *  written permission from Qorvo US, Inc. You must not call products derived from
 *  this software "Qorvo", you must not have "Qorvo" appear in their name, without
 *  the prior permission from Qorvo US, Inc.
 * 6. Qorvo may publish revised or new version of this license from time to time.
 *  No one other than Qorvo US, Inc. has the right to modify the terms applicable
 *  to the software provided under this license.
 * THIS SOFTWARE IS PROVIDED BY QORVO US, INC. "AS IS" AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. NEITHER
 *  QORVO, NOR ANY PERSON ASSOCIATED WITH QORVO MAKES ANY WARRANTY OR
 *  REPRESENTATION WITH RESPECT TO THE COMPLETENESS, SECURITY, RELIABILITY, OR
 *  ACCURACY OF THE SOFTWARE, THAT IT IS ERROR FREE OR THAT ANY DEFECTS WILL BE
 *  CORRECTED, OR THAT THE SOFTWARE WILL OTHERWISE MEET YOUR NEEDS OR EXPECTATIONS.
 * IN NO EVENT SHALL QORVO OR ANYBODY ASSOCIATED WITH QORVO BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 */

#include <stdint.h>
#include "cmsis_os.h"
#include "HAL_timer.h"

#include <stdbool.h>

/* @fn         start_timer(uint32_t *p_timestamp)
 * @brief     save system timestamp (in CLOCKS_PER_SEC)
 * @parm     p_timestamp pointer on current system timestamp
 */
static void start_timer(volatile uint32_t * p_timestamp)
{
    *p_timestamp = HAL_GetTick();
}

/* @fn         check_timer(uint32_t timestamp , uint32_t time)
 * @brief     check if time from current timestamp over expectation
 * @param     [in] timestamp - current timestamp
 * @param     [in] time - time expectation (in CLOCKS_PER_SEC)
 * @return     true - time is over
 *             false - time is not over yet
 */
static bool check_timer(uint32_t timestamp, uint32_t time)
{
    if (HAL_GetTick() - timestamp >= time)
    {
        return (true);
    }

    return (false);
}

/* @fn       init_timer(void) */
static uint32_t init_timer(void)
{
    return 0;
}

/* low-power timer for uwb-stack is not implememnted */
hal_fs_timer_t hal_fs_timer = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,0};

int64_t mcps_get_uptime_us(void)
{
    //TODO This is required since MAC R11.3.0. Needs to be implemented for DW3000
	return 0;
}

/*********************************************************************************/
/** @brief HAL Timer API structure
 */
const struct hal_timer_s Timer = {
    .init = &init_timer,
    .start = &start_timer,
    .check = &check_timer
};
