/**
 * @file      flash_erase.c
 *
 * @brief     
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

#include "HAL_error.h"
#include "appConfig.h"
#include "deca_dbg.h"
#include "deca_error.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Gets the sector of a given address
  * @param  Address for which the sector needs to be retrieved
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t address)
{
  uint32_t sector = 0;

  /** For STM32H7AxI: two independent 1 Mbyte banks of user Flash memory, 
    * each one containing 128 user sectors of 8 Kbytes each. */
  if (IS_FLASH_PROGRAM_ADDRESS_BANK1(address))
  {
    uint32_t address_offset = address - FLASH_BANK1_BASE;
    sector = address_offset / FLASH_SECTOR_SIZE;
  }
  else if (IS_FLASH_PROGRAM_ADDRESS_BANK2(address))
  {
    uint32_t address_offset = address - FLASH_BANK2_BASE;
    sector = address_offset / FLASH_SECTOR_SIZE;
  }
  
  return sector;
}

/* HAL flash erase is done in this function
 * Params_1 : addr- Starting address from where to erase
 * Params_2 : size - Size till which eraase needs to be done
 * Returning status of erase operation*/
int flash_app_erase(eFlashErase_e status)
{
    FLASH_EraseInitTypeDef EraseInitStruct;

    uint32_t firstsector, nb_of_sectors;
    uint32_t sector_error, FLASH_USER_START_ADDR, FLASH_USER_END_ADDR;

    /* Get the 1st sector to erase */
    firstsector = FLASH_SECTOR_2;

    /*FConfig starts from 0x08004000 which is sector 2*/
    FLASH_USER_START_ADDR = FLASH_BANK1_BASE + FLASH_SECTOR_SIZE * (firstsector - FLASH_SECTOR_0);
    FLASH_USER_END_ADDR   = (FLASH_USER_START_ADDR + FCONFIG_SIZE);

    HAL_FLASH_Unlock();

    /* Erase the user Flash area
        (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    /* Get the number of sector to erase from 1st sector*/
    nb_of_sectors = GetSector(FLASH_USER_END_ADDR) - firstsector + 1;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    // EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Banks        = FLASH_BANK_1;
    EraseInitStruct.Sector       = firstsector;
    EraseInitStruct.NbSectors    = nb_of_sectors;

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction
         cache, you have to make sure that these data are rewritten before they are accessed during
         code execution. If this cannot be done safely, it is recommended to flush the caches by setting
         the DCRST and ICRST bits in the FLASH_CR register. */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK)
    {
        diag_printf("Error erase flash 0x%lx error_code 0x%lx\n", sector_error, HAL_FLASH_GetError());
        /*
          Error occurred while sector erase.
          User can add here some code to deal with this error.
          SECTORError will contain the faulty sector and then to know the code error on this sector,
          user can call function 'HAL_FLASH_GetError()'
        */
        error_handler(1, _ERR_Flash_Erase);
    }
    return 0;
}
