/**
 * @file      cmd_fn.c
 *
 * @brief     Collection of executables functions from defined known_commands[]
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
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "reporter.h"
#include "cmd_fn.h"
#include "thisBoard.h"
#include "deca_dbg.h"
#include "usb_uart_tx.h"
#include "cmd.h"
#include "comm_config.h"
#include "rf_tuning_config.h"
#include "HAL_uwb.h"
#include "deca_interface.h"
#include "HAL_uart.h"

//-----------------------------------------------------------------------------
#define MAX_OTP_LENGTH_ADD   0x07F

/***************************************************************************//*
 *
 *                          f_xx "command" FUNCTIONS
 *
 * REG_FN(f_node) macro will create a function
 *
 * const char *f_node(char *text, param_block_t *pbss, int val)
 *
 * */

/**
 * @brief Show all the otp memory
 *
 * */
REG_FN(f_get_otp)
{
    const char * ret = NULL;
    char *str = malloc(MAX_STR_SIZE);
    uint8_t backSleep = 0;
    
    /* Initialize chip */
    struct dwchip_s *dw = hal_uwb.uwbs->dw;
    uint32_t buf;
    struct dwt_otp_read_s otpBuf = {.length = 1,
                                    .address = 0,
                                    .array = &buf };

    if(str)
    {
        int str_len;

        CMD_ENTER_CRITICAL();
        /*Chip may be in sleep*/
        if (hal_uwb.sleep_status_get() == UWB_IS_SLEEPING)
        {
            hal_uwb.wakeup_with_io();
            backSleep = 1;
        }

        str_len = sprintf(str,"{\"OTP FIELD\":[\r\n");
        port_tx_msg((uint8_t*)str, str_len);
        for(int i=0; i<=MAX_OTP_LENGTH_ADD; i++)
        {
            otpBuf.address = i;
            dw->dwt_driver->dwt_ops->ioctl(dw, DWT_OTPREAD, 0, (void*)&otpBuf);
            str_len = sprintf(str,"\"0x%03x\":\"0x%08x\"", (int)i, (int)buf); 
            if(i<(MAX_OTP_LENGTH_ADD))
            {
                str_len += sprintf(&str[str_len], ",\r\n");
            }
            port_tx_msg((uint8_t*)str, str_len);    
        }
        str_len = sprintf(str,"]}\r\n");
        port_tx_msg((uint8_t*)str, str_len);

        if ((hal_uwb.sleep_status_get() != UWB_IS_SLEEPING) && (backSleep))
        { 
            dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENTERSLEEP, 0, NULL);
            hal_uwb.sleep_status_set(UWB_IS_SLEEPING);
        }
        ret = CMD_FN_RET_OK;
        CMD_EXIT_CRITICAL();
    }
    
    free(str);

    return ret;
}



/**
 * @brief set or show current Key & IV parameters in JSON format
 * @param no param - show current Key & IV
 *        correct scanned string - set the params and then show them
 *        incorrect scanned string - error
 *
 * */
REG_FN(f_power)
{
    const char *ret = CMD_FN_RET_OK;

    char *str = malloc(MAX_STR_SIZE);

    if(str)
    {
        /* Display the Key Config */
        int  n, str_len, hlen;

        unsigned int pwr, pgDly;
        dwt_txconfig_t *txConfig = get_dwt_txconfig();

        n = sscanf(text,"%9s 0X%08x 0X%08x", str, &pwr, &pgDly);

        if(n == 3)
        {
            txConfig->power = pwr;
            txConfig->PGdly = pgDly;
        }
        else if(n != 1)
        {
            ret = NULL;
        }

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        str_len = strlen(str);
        str_len += sprintf(&str[str_len],"{\"TX POWER\":{\r\n");

        str_len += sprintf(&str[str_len],"\"PWR\":\"0x%08X\",\r\n", (unsigned int)txConfig->power);
        str_len += sprintf(&str[str_len],"\"PGDLY\":\"0x%08X\"}", (unsigned int)txConfig->PGdly);

        sprintf(&str[2], "%04X", str_len - hlen);   //Insert formatted 4X of length, this will erase first '{'
        str[hlen] = '{';                               //Restore the start bracket
        str_len += sprintf(&str[str_len], "\r\n");
        reporter_instance.print((char*)str, str_len);

        free(str);

        assert(str_len <= MAX_STR_SIZE);
    }

    return (ret);

}


/* Antenna pair structure, to correlate antenna type enum with its string name */
struct antenna_pair_s
{
     antenna_type_e   antenna_type;  /**< Antenna type */
     char            *name;          /**< String to parse/report the correlated type */
};
typedef struct antenna_pair_s antenna_pair_t;

/* Possible antenna type values (enum and string) */
const antenna_pair_t antenna_list[] = {
    { ANT_TYPE_NONE       ,"NONE" },
    { ANT_TYPE_MAN5       ,"MAN5" },
    { ANT_TYPE_CPWING5    ,"CPWING5" },
    { ANT_TYPE_CPWING9    ,"CPWING9" },
    { ANT_TYPE_MONALISA5  ,"MONALISA5" },
    { ANT_TYPE_MONALISA9  ,"MONALISA9" },
    { ANT_TYPE_JOLIE5     ,"JOLIE5" },
    { ANT_TYPE_JOLIE9     ,"JOLIE9" },
    { ANT_TYPE_CUSTOM     ,"CUSTOM" },
    { 0                   ,NULL },
};

#define NUMBER_OF_ANT_PORTS   (int)sizeof(antenna_t)
/**
 * @brief sets or show the current Antenna Type
 * @param no param - show current antenna type
 *        correct scanned string - set the antenna type and then show it
 *        incorrect scanned string - error
 *
 * */
REG_FN(f_antenna)
{
    const char *ret = CMD_FN_RET_OK;
    char        argv[NUMBER_OF_ANT_PORTS][12];

    int  n;
    int  arg_index, ant_index;
    bool bad_type;

    antenna_type_e *port;
    antenna_type_e  antenna_type[NUMBER_OF_ANT_PORTS];

    rf_tuning_t *rf_tuning = get_rf_tuning_config();

    n = sscanf(text,"%*s %9s %9s %9s %9s", argv[0], argv[1], argv[2], argv[3] );
    diag_printf("\r\n");  // New line to start response after command

    if (n <= NUMBER_OF_ANT_PORTS)
    {
        /* Option to show possible values if the command is "VALUES" */
        if (strcmp(argv[0], "VALUES") == 0)
        {
            /* Print possible values for the "antenna" command */
            diag_printf("ANTENNA_TYPE POSSIBLE VALUES:\r\n");

            ant_index = 0;
            while (antenna_list[ant_index].name != NULL)
            {
                diag_printf("- %s\r\n", antenna_list[ant_index].name);
                ant_index++;
            }
            diag_printf("\r\n");
        }
        else if (n > 0)
        {
            /* Check values given for each antenna port */
            for (arg_index = 0; arg_index < n; arg_index++)
            {
                bad_type = true;
                ant_index = 0;

                while (antenna_list[ant_index].name != NULL)
                {
                    if (strcmp(argv[arg_index], antenna_list[ant_index].name) == 0)
                    {
                        antenna_type[arg_index] = antenna_list[ant_index].antenna_type;
                        bad_type = false;
                        break;
                    }
                    ant_index++;
                }

                if (bad_type)
                    break;
            }
            /* Check/report for invalid types... */
            if (bad_type)
            {
                diag_printf("INVALID ANTENNA_TYPE: %s\r\n", argv[arg_index]);
                ret = NULL;
            }
            else
            {
                /* ...if valid, change antenna types and take additional actions */
                port = &rf_tuning->antenna.port1;
                for (ant_index = 0; ant_index < n; ant_index++)
                {
                    *port = antenna_type[ant_index];
                    port++;
                }

                /* Additional action - Set Antenna Delay */
                //rf_tuning->antTx_a = (uint16_t)(0.5 * DEFAULT_ANTD);
                //rf_tuning->antRx_a = (uint16_t)(0.5 * DEFAULT_ANTD);
                //rf_tuning->antRx_b = (uint16_t)(0.5 * DEFAULT_ANTD);
            }
        }
    }
    else
    {
        diag_printf("INVALID COMMAND FORMAT!\r\n");
        ret = NULL;
    }

    /* Display Antenna Type */
    if (ret != NULL)
    {
        diag_printf("CURRENT ANTENNA_TYPE:\r\n");

        port = &rf_tuning->antenna.port1;
        for (ant_index = 0; ant_index < NUMBER_OF_ANT_PORTS; ant_index++)
        {
            diag_printf("PORT%d: %s\r\n", ant_index + 1, antenna_list[*port].name);
            port++;
        }
    }

    return (ret);
}

REG_FN(f_uart)
{
    bool uartEn = get_uartEn();
    if (uartEn && !val)
    {
        deca_uart_close();
    }
    else if (!uartEn && val)
    {
        deca_uart_init();
    }
    set_uartEn((uint8_t)(val) == 1);
    return (CMD_FN_RET_OK);
}

/**
  * @}
  */


//-----------------------------------------------------------------------------

/** end f_xx command functions */
const char COMMENT_GETOTP[]= {"Retrieve OTP memory.\r\nUsage: To get otp memory values \"getotp\" "};
const char COMMENT_TXPOWER[] = {"Tx Power settings.\r\nUsage: To see Tx power \"TXPOWER\". To set the Tx power \"TXPOWER 0x<POWER_HEX> 0x<PGDLY_HEX>\""};
const char COMMENT_ANTENNA[] = {"Sets Antenna Type.\r\nUsage: To see Antenna \"ANTENNA\". To set the current antenna type for each port \"ANTENNA <PORT1> <PORT2>...\". To see possible values \"antenna values\"."};
const char COMMENT_UART[] = {"Usage: To initialize selected UART \"UART <DEC>\""};

//-----------------------------------------------------------------------------
/** list of known commands:
 * NAME, allowed_MODE,     REG_FN(fn_name)
 * */
const struct command_s known_commands_dw3xxx [] __attribute__((section(".known_commands_service")))= {
    {"GETOTP",  mCmdGrp1 | mIDLE,  f_get_otp,               COMMENT_GETOTP},
    {"TXPOWER", mCmdGrp1 | mIDLE,  f_power,                 COMMENT_TXPOWER},
    {"ANTENNA", mCmdGrp1 | mIDLE,  f_antenna,               COMMENT_ANTENNA},
};

const struct command_s known_commands_idle_uart [] __attribute__((section(".known_commands_ilde")))= {
    {"UART",    mCmdGrp1 | mIDLE,  f_uart,                  COMMENT_UART},
};
