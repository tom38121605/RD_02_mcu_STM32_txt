/**
 *  @file     audio_tx_fn.c
 *
 *  @brief    Collection of executables functions from defined known_commands[]
 *
 * @author    Decawave Applications
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

#include <string.h>
#include <stdio.h>

/* sscanf not included in stdio.h... */
int sscanf(const char *str, const char *format, ...);

#include "cmd_fn.h"
#include "cmd.h"
#include "EventManager.h"
#include "common_audio.h"
#include "reporter.h"

const char COMMENT_AUDIO_TX_OPT    []={"AUDIO TX Options -----"};
const char COMMENT_AUDIO_TX_STAT   []={"Displays the statistics inside the Audio TX application."};
const char COMMENT_AUDIO_TX        []={"Listen for the UWB packjets using the UWB configuration.\r\n"};

extern const app_definition_t helpers_app_audio_tx[];

/**
 * @brief   defaultTask will start audio_tx user application
 *
 * */
REG_FN(f_audio_tx)
{
    app_definition_t *app_ptr = (app_definition_t *)&helpers_app_audio_tx[0];
    EventManagerRegisterApp((void *)&app_ptr);

    return (CMD_FN_RET_OK);
}

REG_FN(f_audio_tx_stat)
{
    char *str = malloc(MAX_STR_SIZE);

    if(str)
    {
        CMD_ENTER_CRITICAL();
        int  hlen;
        audio_info_t *info = getAudioInfoPtr();
        /** Listener TX Event Counts object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"Audio TX\":{\r\n");
        sprintf(&str[strlen(str)],"\"package_size\":%d}}",(int)info->package_size);
        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        reporter_instance.print((char*)str, strlen(str));

        free(str);

        CMD_EXIT_CRITICAL();
    }
    return(CMD_FN_RET_OK);
}

const struct command_s known_app_audio_tx [] __attribute__((section(".known_commands_app")))= {
    {"AUDIO_TX", mCmdGrp2 | mIDLE,  f_audio_tx,             COMMENT_AUDIO_TX},
};

const struct command_s known_commands_audio_tx [] __attribute__((section(".known_app_subcommands")))= {
    {NULL,      mCmdGrp0 | mAPP,                    NULL,    COMMENT_AUDIO_TX_OPT},
    {"AUDIO_TX_STAT",   mCmdGrp1 | mAPP | APP_LAST_SUB_CMD, f_audio_tx_stat, COMMENT_AUDIO_TX_STAT  },
};