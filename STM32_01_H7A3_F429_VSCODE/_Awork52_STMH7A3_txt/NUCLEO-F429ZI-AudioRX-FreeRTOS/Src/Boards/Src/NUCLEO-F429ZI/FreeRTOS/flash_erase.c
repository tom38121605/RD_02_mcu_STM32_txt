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
#include "stm32f4xx_hal.h"

//#define FLASH_USER_START_ADDR       ADDR_FLASH_SECTOR_6   /* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR         ADDR_FLASH_SECTOR_7  +  GetSectorSize(ADDR_FLASH_SECTOR_7) -1

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Gets the sector of a given address
  * @param  Address for which the sector needs to be retrieved
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t address)
{
  uint32_t sector = 0;

  if((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else if((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;
  }
  else if((address < ADDR_FLASH_SECTOR_13) && (address >= ADDR_FLASH_SECTOR_12))
  {
    sector = FLASH_SECTOR_12;
  }
  else if((address < ADDR_FLASH_SECTOR_14) && (address >= ADDR_FLASH_SECTOR_13))
  {
    sector = FLASH_SECTOR_13;
  }
  else if((address < ADDR_FLASH_SECTOR_15) && (address >= ADDR_FLASH_SECTOR_14))
  {
    sector = FLASH_SECTOR_14;
  }
  else if((address < ADDR_FLASH_SECTOR_16) && (address >= ADDR_FLASH_SECTOR_15))
  {
    sector = FLASH_SECTOR_15;
  }
  else if((address < ADDR_FLASH_SECTOR_17) && (address >= ADDR_FLASH_SECTOR_16))
  {
    sector = FLASH_SECTOR_16;
  }
  else if((address < ADDR_FLASH_SECTOR_18) && (address >= ADDR_FLASH_SECTOR_17))
  {
    sector = FLASH_SECTOR_17;
  }
  else if((address < ADDR_FLASH_SECTOR_19) && (address >= ADDR_FLASH_SECTOR_18))
  {
    sector = FLASH_SECTOR_18;
  }
  else if((address < ADDR_FLASH_SECTOR_20) && (address >= ADDR_FLASH_SECTOR_19))
  {
    sector = FLASH_SECTOR_19;
  }
  else if((address < ADDR_FLASH_SECTOR_21) && (address >= ADDR_FLASH_SECTOR_20))
  {
    sector = FLASH_SECTOR_20;
  }
  else if((address < ADDR_FLASH_SECTOR_22) && (address >= ADDR_FLASH_SECTOR_21))
  {
    sector = FLASH_SECTOR_21;
  }
  else if((address < ADDR_FLASH_SECTOR_23) && (address >= ADDR_FLASH_SECTOR_22))
  {
    sector = FLASH_SECTOR_22;
  }
  else
  {
      sector = FLASH_SECTOR_23;
  }

  return sector;
}
/**
  * @brief  Gets sector Size
  * @param  Sector
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;
  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) ||\
     (Sector == FLASH_SECTOR_3) || (Sector == FLASH_SECTOR_12) || (Sector == FLASH_SECTOR_13) ||\
     (Sector == FLASH_SECTOR_14) || (Sector == FLASH_SECTOR_15))
  {
    sectorsize = 16 * 1024;
  }
  else if((Sector == FLASH_SECTOR_4) || (Sector == FLASH_SECTOR_16))
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
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

    if(status == Netboot_Erase)
    {
        FLASH_USER_START_ADDR = ADDR_FLASH_SECTOR_6;
        FLASH_USER_END_ADDR = ADDR_FLASH_SECTOR_7  +  GetSectorSize(ADDR_FLASH_SECTOR_7) -1;
    }
    else // if(status == CONFIG_ERASE)
    {
        FLASH_USER_START_ADDR = ADDR_FLASH_SECTOR_1;
        FLASH_USER_END_ADDR = (ADDR_FLASH_SECTOR_1 + FCONFIG_SIZE);
    }

    HAL_FLASH_Unlock();

      /* Erase the user Flash area
        (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

      /* Get the 1st sector to erase */
      firstsector = GetSector  (FLASH_USER_START_ADDR);
      /* Get the number of sector to erase from 1st sector*/
      nb_of_sectors = GetSector(FLASH_USER_END_ADDR) - firstsector + 1;

      /* Fill EraseInit structure*/
      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
      EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
      EraseInitStruct.Sector        = firstsector;
      EraseInitStruct.NbSectors     = nb_of_sectors;

      /* Note: If an erase operation in Flash memory also concerns data in the data or instruction
         cache, you have to make sure that these data are rewritten before they are accessed during
         code execution. If this cannot be done safely, it is recommended to flush the caches by setting
         the DCRST and ICRST bits in the FLASH_CR register. */
      if(HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK)
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
