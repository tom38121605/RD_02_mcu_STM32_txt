/**
 * @file    deca_SPI.c
 *
 * @brief   Hardware abstraction layer for SPI
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

#include <stdbool.h>
#include "assert.h"
#include "HAL_uwb.h"
#include "HAL_SPI.h"
#include "HAL_error.h"
#include "minmax.h"
#include "main.h"

//-----------------------------------------------------------------------------

static void spi_slow_rate_(void *handler);
static void spi_fast_rate_(void *handler);
static void spi_cs_low_(void *handler);
static void spi_cs_high_(void *handler);
static int readfromspi_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer);
static int writetospi_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer);

#if defined(DMA_SPI)
#define SPI_DMA_MIN        (32)
typedef enum
{
    TXRX_BLOCK_START = 0,
    TXRX_NONBLOCK_START,
    TXRX_COMPLETE
}spi_transmit_flag;
#endif

struct spi_handle_s
{
    SPI_HandleTypeDef       *phspi;
    uint32_t                prescaler_slow;
    uint32_t                prescaler_fast;
    uint32_t                csPin;
    GPIO_TypeDef            *csPort;
    __IO HAL_LockTypeDef    Lock;
    __IO uint32_t           TxComplete;
    __IO uint32_t           RxComplete;
};

typedef struct spi_handle_s spi_handle_t;


extern SPI_HandleTypeDef hspi1;

static spi_handle_t spi_handler1 =
{//SPI1 clocked from 72MHz
    .phspi          = &hspi1,
    .prescaler_slow = SPI_BAUDRATEPRESCALER_16,  // 4.5 MHz
    .prescaler_fast = SPI_BAUDRATEPRESCALER_2,   // 36 MHz
    .csPin          = DW_CS_Pin,
    .csPort         = DW_CS_GPIO_Port,
    .Lock           = HAL_UNLOCKED,
#if defined(DMA_SPI)
    .TxComplete     = TXRX_COMPLETE,
    .RxComplete     = TXRX_COMPLETE,
#endif
};

static const struct spi_s spi1 =
{
    .cs_high = spi_cs_high_,
    .cs_low = spi_cs_low_,
    .slow_rate = spi_slow_rate_,
    .fast_rate = spi_fast_rate_,
    .read = readfromspi_,
    .write = writetospi_,
    .write_with_crc = NULL,
    .handler = &spi_handler1
};

static void spi_cs_low_(void *handler)
{   
    spi_handle_t *spi_handler = handler;
    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_RESET);
}

static void spi_cs_high_(void *handler)
{  
    spi_handle_t *spi_handler = handler;
    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET);
}

//-----------------------------------------------------------------------------

/* @fn  spi_slow_rate
 * @brief sets slow SPI clock speed for the DW chip
 *        left for compatibility.
 * */
static void spi_slow_rate_(void *handler)
{
    spi_handle_t *spi_handler = handler;
    spi_handler->phspi->Init.BaudRatePrescaler = spi_handler->prescaler_slow;
    __DSB();
    HAL_SPI_Init(spi_handler->phspi);
}

/* @fn      spi_fast_rate
 * @brief   sets High SPI clock speed for the DW chip
 * */
static void spi_fast_rate_(void *handler)
{
    spi_handle_t *spi_handler = handler;
    spi_handler->phspi->Init.BaudRatePrescaler = spi_handler->prescaler_fast;
    __DSB();
    HAL_SPI_Init(spi_handler->phspi);
}

/****************************************************************************//**
 *
 *                              DWxxx SPI section
 *
 *******************************************************************************/
//==============================================================================
const struct spi_s *init_spi(const spi_port_config_t *dw3000_spi_port_cfg)
{
    return &spi1;
}

#if !defined(DMA_SPI)
/*!
 * @fn  openspi()
 * @brief
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
}


/*!
 * @fn  closespi()
 * @brief
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
}

/*!
 * @fn  writetospi()
 * @brief
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    spi_handle_t *spi_handler = handler;

    /* Blocking: Check whether the previous transfer has been finished */
    while(spi_handler->Lock == HAL_LOCKED);
    __HAL_LOCK(spi_handler);                    //"return HAL_BUSY;" if locked

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)&headerBuffer[0], headerLength, 10);    /* Send header in polling mode */
    HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)&bodyBuffer[0], bodyLength, 10);        /* Send data in polling mode */

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET); /**< Put chip select line high */

    __HAL_UNLOCK(spi_handler);
    return 0;
}

/**---------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
    spi_handle_t *spi_handler = handler;

    /* Blocking: Check whether the previous transfer has been finished */
    while(spi_handler->Lock == HAL_LOCKED);
    __HAL_LOCK(spi_handler);    //"return HAL_BUSY;" if locked

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)&headerBuffer[0], headerLength, 10);    /* Send header in polling mode */
    HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)&bodyBuffer[0], bodyLength, 10);        /* Send data in polling mode */
    HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)&crc8, 1, 10);                          /* Send data in polling mode */

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET); /**< Put chip select line high */

    __HAL_UNLOCK(spi_handler);
    return 0;
} // end writetospiwithcrc()


/**---------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer)
{
    spi_handle_t *spi_handler = handler;

    /* Blocking: Check whether the previous transfer has been finished */
    while(spi_handler->Lock == HAL_LOCKED);
    __HAL_LOCK(spi_handler);    //"return HAL_BUSY;" if locked

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_RESET); /**< Put chip select line low */

    /* Send header */
    for(int i=0; i<headerLength; i++)
    {
        HAL_SPI_Transmit(spi_handler->phspi, (uint8_t*)&headerBuffer[i], 1, HAL_MAX_DELAY); //No timeout
    }

    /* for the data buffer use LL functions directly as the HAL SPI read function
     * has issue reading single bytes */
    while(readlength-- > 0)
    {
        /* Wait until TXE flag is set to send data */
        while(__HAL_SPI_GET_FLAG(spi_handler->phspi, SPI_FLAG_TXE) == RESET)
        {
        }

        spi_handler->phspi->Instance->DR = 0; /* set the output to 0 (MOSI), this is necessary to keep MOSI line low.
                                                  e.g. when waking up DW3000 from DEEPSLEEP via dwt_spicswakeup() function.
                                                */

        /* Wait until RXNE flag is set to read data */
        while(__HAL_SPI_GET_FLAG(spi_handler->phspi, SPI_FLAG_RXNE) == RESET)
        {
        }

        (*readBuffer++) = spi_handler->phspi->Instance->DR;  //copy data read form (MISO)
    }

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET); /**< Put chip select line high */

    /* Process Unlocked */
    __HAL_UNLOCK(spi_handler);

    return 0;
} // end readfromspi()


#else

//-----------------------------------------------------------------------------

/*
 * @brief
 * Low level abstract function to read from the SPI using DMA (But in blocking mode)
 * Takes two separate byte buffers for write header and read data
 * returns 0 for success, or stuck in error handler
 *
 *  The best results we can achieve: 36MHz:
 *  reading of 0x3000 accumulator would take 2.7ms
 *  reading of DevID (5bytes) 4.2us.
 *
 * Note this implementation should not be used for DWxxx dwt_spicswakeup() function.
 * the Tx Buffer is not zeroed, so it will be the garbage on the MOSI line.
 * TODO: remove dwt_spicswakeup() from the driver code
 *
 * */
static int readfromspi_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer)
{
    spi_handle_t *spi_handler = handler;

    if( (spi_handler->phspi->Instance != SPI1) )
    {
        error_handler(1, _ERR_SPI_DMA);
    }

    while(spi_handler->Lock == HAL_LOCKED);
    __HAL_LOCK(spi_handler);    //"return HAL_BUSY;" if locked

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_RESET);

    if(headerLength + readlength < SPI_DMA_MIN)
    {
        // Send without DMA to achieve higher throughput if data length less then SPI_DMA_MIN
        /* Send header */
        HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)headerBuffer, headerLength, HAL_MAX_DELAY);
        if (readlength > 0)
        {
            HAL_SPI_Receive(spi_handler->phspi, (uint8_t *)readBuffer, readlength, HAL_MAX_DELAY);
        }
    }
    else
    {
        /* Send header in polling mode */
        HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)headerBuffer, headerLength, HAL_MAX_DELAY);
        spi_handler->RxComplete = TXRX_BLOCK_START;
        __DSB();
        __ISB();
        HAL_SPI_Receive_DMA(spi_handler->phspi, readBuffer, readlength);
        while (spi_handler->RxComplete != TXRX_COMPLETE);
    }
    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET);
    __HAL_UNLOCK(spi_handler);

    return _NO_ERR;
}

/*
 * @brief
 * Low level abstract function to write to the SPI using DMA (blocking mode)
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or stuck in error handler
 * */
static int writetospi_(void *handler, uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    spi_handle_t *spi_handler = handler;

    if( (spi_handler->phspi->Instance != SPI1) )
    {
        error_handler(1, _ERR_SPI_DMA);
    }

    while(spi_handler->Lock == HAL_LOCKED);
    __HAL_LOCK(spi_handler);    //"return HAL_BUSY;" if locked

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_RESET);

    if (headerLength + bodyLength < SPI_DMA_MIN)
    {
        // Send without DMA to achieve higher throughput if data length less then SPI_DMA_MIN
        HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)headerBuffer, headerLength, 10);    /* Send header in polling mode */
        if (bodyLength > 0)
        {
            HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)bodyBuffer, bodyLength, 10);        /* Send data in polling mode */
        }
    }
    else
    {
        /* Send header in polling mode */
        HAL_SPI_Transmit(spi_handler->phspi, (uint8_t *)headerBuffer, headerLength, 10);

        if (bodyLength > 0)
        {
            spi_handler->TxComplete = TXRX_BLOCK_START;
            __DSB();
            __ISB();

            /* Send body in polling mode */
            HAL_SPI_Transmit_DMA(spi_handler->phspi, (uint8_t *)bodyBuffer, bodyLength);
            while (spi_handler->TxComplete != TXRX_COMPLETE);
        }
    }

    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET);
    __HAL_UNLOCK(spi_handler);

    return _NO_ERR;
}

//-----------------------------------------------------------------------------

/**
  * @brief  Function called from DMA2Stream2 IRQ Handler when Rx transfer is completed
  * @param
  * @retval None
  */
void DMA_UWB_ReceiveComplete_cb(void)
{
    spi_handle_t *spi_handler = &spi_handler1;
    spi_handler->RxComplete = TXRX_COMPLETE;
}

/**
  * @brief  Function called from DMA2Stream2 IRQ Handler when Tx&RX transfer is completed
  * @param
  * @retval None
  */
void DMA_UWB_SendReceiveComplete_cb(void)
{
    spi_handle_t *spi_handler = &spi_handler1;
    spi_handler->RxComplete = TXRX_COMPLETE;
    spi_handler->TxComplete = TXRX_COMPLETE;
}

/**
  * @brief  Function called from DMA2Stream2 IRQ Handler when Tx transfer is completed
  * @param
  * @retval None
  */
void DMA_UWB_SendComplete_cb(void)
{
    spi_handle_t *spi_handler = &spi_handler1;
    spi_handler->TxComplete = TXRX_COMPLETE;
}

/**
  * @brief  Function called from DMA2Stream2 IRQ Handler when transfer error
  * @param
  * @retval None
  */
void SPI_UWB_TransferError_Callback(void)
{
    spi_handle_t *spi_handler = &spi_handler1;
    spi_handler->TxComplete = TXRX_COMPLETE;
    spi_handler->RxComplete = TXRX_COMPLETE;
    HAL_GPIO_WritePin(spi_handler->csPort, spi_handler->csPin, GPIO_PIN_SET);
    __HAL_UNLOCK(spi_handler);
    error_handler(1, _ERR_SPI_DMA);
}

#endif

/****************************************************************************//**
 *
 *                                 END OF SPI section
 *
 *******************************************************************************/
 
