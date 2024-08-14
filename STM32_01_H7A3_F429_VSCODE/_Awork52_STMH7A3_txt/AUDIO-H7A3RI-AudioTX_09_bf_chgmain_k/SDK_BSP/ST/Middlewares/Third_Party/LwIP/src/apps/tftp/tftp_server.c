/****************************************************************//**
 *
 * @file tftp_server.c
 *
 * @author   Logan Gunthorpe <logang@deltatee.com>
 *           Dirk Ziegelmeier <dziegel@gmx.de>
 *
 * @brief    Trivial File Transfer Protocol (RFC 1350)
 *
 * Copyright (c) Deltatee Enterprises Ltd. 2013
 * All rights reserved.
 *
 ********************************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification,are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Logan Gunthorpe <logang@deltatee.com>
 *         Dirk Ziegelmeier <dziegel@gmx.de>
 *
 */

/**
 *
 * @brief Support of Decawave TDOA anchor TFTP network updater
 *
 * This is simple TFTP server for the lwIP raw API.
 */

#include "lwip/apps/tftp_server.h"
#include "bootload.h"

#if LWIP_UDP

#include "lwip/udp.h"
#include "lwip/timeouts.h"
#include "lwip/debug.h"
#include "lwip/netif.h"
#include "app_ethernet.h"
#include <string.h>

#define TFTP_MAX_PAYLOAD_SIZE 512
#define TFTP_HEADER_LENGTH    4

#define TFTP_RRQ   1
#define TFTP_WRQ   2
#define TFTP_DATA  3
#define TFTP_ACK   4
#define TFTP_ERROR 5

extern char gfileName[];

enum tftp_error {
  TFTP_ERROR_FILE_NOT_FOUND    = 1,
  TFTP_ERROR_ACCESS_VIOLATION  = 2,
  TFTP_ERROR_DISK_FULL         = 3,
  TFTP_ERROR_ILLEGAL_OPERATION = 4,
  TFTP_ERROR_UNKNOWN_TRFR_ID   = 5,
  TFTP_ERROR_FILE_EXISTS       = 6,
  TFTP_ERROR_NO_SUCH_USER      = 7
};

enum packet_type {RRQ = 1, WRQ = 2, DATA = 3, ACK  = 4};

typedef struct data_packet
{
    short unsigned opcode;  /* 3 */
    u16_t block_number;
    unsigned char data[512];
} data_packet;

static void tftp_done(void);
static void tftp_get_file(char *img_file);

typedef struct generic_packet
{
    short unsigned opcode;
    char info[514];
} generic_packet;

typedef struct ack_packet
{
    short unsigned opcode;  /* 4 */
    short unsigned block_number;
} ack_packet;

struct tftp_state {
  const struct tftp_context *ctx;
  void *handle;
  struct pbuf *last_data;
  struct udp_pcb *upcb;
  ip_addr_t addr;
  u16_t port;
  int timer;
  int last_pkt;
  u16_t blknum;
  u8_t retries;
  u8_t mode_write;
};

static struct udp_pcb   *tftp_pcb;
struct ip_addr  tftp_server;
static struct tftp_state tftp_state;

extern volatile net_app_state_e app_mode;

/* Data array declared globally which needs to store
 * the data packets received from TFTP client */
static uint32_t g_data_32[DATA_PACKET_WORD_SIZE_LIMIT];

/* Statring address declared globally since it will be
 * taken as reference from where flashing needs to happen */
static uint32_t g_address_sector = ADDR_FLASH_SECTOR_6;

static void tftp_tmr(void* arg);

static void
close_handle(void)
{
  tftp_state.port = 0;
  ip_addr_set_any(0, &tftp_state.addr);

  if(tftp_state.last_data != NULL) {
    pbuf_free(tftp_state.last_data);
    tftp_state.last_data = NULL;
  }

  sys_untimeout(tftp_tmr, NULL);

  if (tftp_state.handle) {
    tftp_state.ctx->close(tftp_state.handle);
    tftp_state.handle = NULL;
    LWIP_DEBUGF(TFTP_DEBUG | LWIP_DBG_STATE, ("tftp: closing\n"));
  }
}

static void
send_error(const ip_addr_t *addr, u16_t port, enum tftp_error code, const char *str)
{
  int str_length = strlen(str);
  struct pbuf* p;
  u16_t* payload;

  p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(TFTP_HEADER_LENGTH + str_length + 1), PBUF_RAM);
  if(p == NULL) {
    return;
  }

  payload = (u16_t*) p->payload;
  payload[0] = PP_HTONS(TFTP_ERROR);
  payload[1] = lwip_htons(code);
  MEMCPY(&payload[2], str, str_length + 1);

  udp_sendto(tftp_state.upcb, p, addr, port);
  pbuf_free(p);
}

static void
send_ack(u16_t blknum,  u16_t port)
{
  struct pbuf* p;
  u16_t* payload;

  p = pbuf_alloc(PBUF_TRANSPORT, TFTP_HEADER_LENGTH, PBUF_RAM);
  if(p == NULL) {
    return;
  }
  payload = (u16_t*) p->payload;

  payload[0] = PP_HTONS(TFTP_ACK);
  payload[1] = lwip_htons(blknum);
  udp_sendto(tftp_state.upcb, p, &tftp_state.addr , port);
  pbuf_free(p);
}

static void
resend_data(void)
{
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, tftp_state.last_data->len, PBUF_RAM);
  if(p == NULL) {
    return;
  }

  if(pbuf_copy(p, tftp_state.last_data) != ERR_OK) {
    pbuf_free(p);
    return;
  }

  pbuf_free(p);
}

static void
send_data(void)
{
  u16_t *payload;
  int ret;

  if(tftp_state.last_data != NULL) {
    pbuf_free(tftp_state.last_data);
  }

  tftp_state.last_data = pbuf_alloc(PBUF_TRANSPORT, TFTP_HEADER_LENGTH + TFTP_MAX_PAYLOAD_SIZE,
          PBUF_RAM);
  if(tftp_state.last_data == NULL) {
    return;
  }

  payload = (u16_t *) tftp_state.last_data->payload;
  payload[0] = PP_HTONS(TFTP_DATA);
  payload[1] = lwip_htons(tftp_state.blknum);

  ret = tftp_state.ctx->read(tftp_state.handle, &payload[2], TFTP_MAX_PAYLOAD_SIZE);
  if (ret < 0) {
    send_error(&tftp_state.addr, tftp_state.port, TFTP_ERROR_ACCESS_VIOLATION,
            "Error occured while reading the file.");
    close_handle();
    return;
  }

  pbuf_realloc(tftp_state.last_data, (u16_t)(TFTP_HEADER_LENGTH + ret));
  resend_data();
}

static void
tftp_data_cb(void *arg, struct udp_pcb *udp_pcb, struct pbuf *pkt_buf, const ip_addr_t *ipad, u16_t port)
{
    data_packet     *data;

    if(pkt_buf->len > 0)
    {
        app_mode = NET_APP_TFTP_FLASHING;

        int data_index=0, incr_index=0;

        ip_addr_copy(tftp_state.addr, *ipad);
        ip4_addr_copy(tftp_server,*ipad);

        data = (data_packet *)pkt_buf->payload;

        u16_t *sbuf = (u16_t *) pkt_buf->payload;
        tftp_state.blknum = lwip_ntohs(sbuf[1]);

        /* Rearrangning the data packet into word size */
        while(data_index < DATA_PACKET_WORD_SIZE_LIMIT && incr_index < DATA_PACKET_SIZE_LIMIT)
        {
            g_data_32[data_index] = data->data[incr_index + ZEROTH_INDEX] |
                    (data->data[incr_index + FIRST_INDEX] << SHIFT_EIGHT) |
                    (data->data[incr_index + SECOND_INDEX] << SHIFT_SIXTEEN |
                    (data->data[incr_index + THIRD_INDEX] << SHIFT_TWENTY_FOUR));
            incr_index = incr_index + INCREMENT_WORD;
            data_index++;
        }


        if( ntohs(data->opcode) == ERROR)
        {
            printf("No Software Update\n");
        }
        else
        {
            send_ack(tftp_state.blknum,port);

            for(data_index = 0; data_index < 128;)
            {
                if(HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, g_address_sector,
                             g_data_32[data_index++]))
                {
                    printf("ERROR WHILE FLASHING !!!]\n");
                    bootloader(NETBOOT_FLASH);
                }

                /* Address being incremented for the next word of packet */
                g_address_sector += INCREMENT_WORD;
            }

            if(pkt_buf->len < (DATA_PACKET_SIZE_LIMIT + INCREMENT_WORD))
            {
                tftp_done();
            }

            pbuf_free(pkt_buf);
    }

    }


}

static void
tftp_tmr(void* arg)
{
  LWIP_UNUSED_ARG(arg);

  tftp_state.timer++;

  if (tftp_state.handle == NULL) {
    return;
  }

  sys_timeout(TFTP_TIMER_MSECS, tftp_tmr, NULL);

  if ((tftp_state.timer - tftp_state.last_pkt) > (TFTP_TIMEOUT_MSECS / TFTP_TIMER_MSECS)) {
    if ((tftp_state.last_data != NULL) && (tftp_state.retries < TFTP_MAX_RETRIES)) {
      LWIP_DEBUGF(TFTP_DEBUG | LWIP_DBG_STATE, ("tftp: timeout, retrying\n"));
      resend_data();
      tftp_state.retries++;
    } else {
      LWIP_DEBUGF(TFTP_DEBUG | LWIP_DBG_STATE, ("tftp: timeout\n"));
      close_handle();
    }
  }
}

/** @ingroup tftp
 * Initialize TFTP server.
 * @param ctx TFTP callback struct
 */
err_t
tftp_init(const struct tftp_context *ctx)
{
  err_t ret;

  tftp_pcb = udp_new();
  if (tftp_pcb == NULL) {
    return ERR_MEM;
  }

  ret = udp_bind(tftp_pcb, IP_ADDR_ANY, 0);
  if (ret != ERR_OK) {
    udp_remove(tftp_pcb);
    return ret;
  }

  tftp_state.handle    = NULL;
  tftp_state.port      = 0;
  tftp_state.ctx       = ctx;
  tftp_state.timer     = 0;
  tftp_state.last_data = NULL;
  tftp_state.upcb      = tftp_pcb;

  udp_recv(tftp_pcb, tftp_data_cb, NULL);

  tftp_get_file(gfileName);

  return ERR_OK;
}

/* @brief request for the file to update
 *
 * */
void tftp_get_file(char *img_file)
{
    struct pbuf     *tx_pbuf;
    generic_packet  read_req;
    int read_req_len;

    read_req_len= 2 + strlen(img_file) + 1 + 5 + 1 ;

    printf("RRQ Len: %d \n", read_req_len);

    read_req.opcode = htons(RRQ);
    sprintf(read_req.info, "%s%c%s%c",img_file, '\0',"octet", '\0');

    tx_pbuf = pbuf_alloc(PBUF_TRANSPORT, read_req_len, PBUF_RAM);

    if(tx_pbuf)
    {
        printf("Requesting Application Image: %s\n", img_file);

        memcpy(tx_pbuf->payload, (unsigned char*)&read_req, read_req_len);
        udp_sendto(tftp_pcb, tx_pbuf, (ip_addr_t *)&tftp_server.addr , 69);

        app_mode = NET_APP_TFTP_FILE_REQ;

        pbuf_free(tx_pbuf);
    }
    else
    {
        printf("Fail: pbuf_alloc \n");
    }
}


void tftp_done(void)
{
    app_mode = NET_APP_END_FLASHING_OK; /*!< this will finalize the update process */
}

#endif /* LWIP_UDP */


