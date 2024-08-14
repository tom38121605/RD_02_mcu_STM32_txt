/**
 ******************************************************************************
 * @file    audio_speaker_node.c
 * @author  MCD Application Team
 * @brief   speaker node implementation.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019  STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#if USE_USB_AUDIO_PLAYBACK

#include <string.h>

#include "audio_speaker_node.h"
#include "audio_tx.h"
#include "task_audio_tx.h"
#include "usb_audio.h"
#include "usbd_audio.h"

/* Private defines -----------------------------------------------------------*/
#define SPEAKER_CMD_STOP                        1
#define SPEAKER_CMD_EXIT                        2
#define SPEAKER_CMD_CHANGE_FREQUENCE            4
#define VOLUME_DB_256_TO_PERCENT(volume_db_256) ((uint8_t)((((int)(volume_db_256)-VOLUME_SPEAKER_MIN_DB_256) * 100) / (VOLUME_SPEAKER_MAX_DB_256 - VOLUME_SPEAKER_MIN_DB_256)))

#define AUDIO_SPEAKER_INJECTION_LENGTH(audio_desc)     AUDIO_MS_PACKET_SIZE((audio_desc)->frequency, (audio_desc)->channels_count, (audio_desc)->resolution)
#define AUDIO_SPEAKER_MAX_INJECTION_LENGTH(audio_desc) AUDIO_SPEAKER_INJECTION_LENGTH(audio_desc)

/* alt buffer max size */
#define SPEAKER_ALT_BUFFER_SIZE ((USB_AUDIO_CONFIG_PLAY_FREQ_MAX + 999) / 1000) * 2 * 4 * 2

//#define DEBUG_SPEAKER_NODE 1 /* define when debug is required*/
#ifdef DEBUG_SPEAKER_NODE
#define SPEAKER_DEBUG_BUFFER_SIZE 1000
#endif /*DEBUG_SPEAKER_NODE*/

/* Private function prototypes -----------------------------------------------*/
static int8_t AUDIO_SpeakerDeInit(uint32_t node_handle);
static int8_t AUDIO_SpeakerStart(AUDIO_CircularBuffer_t *buffer, uint32_t node_handle);
static int8_t AUDIO_SpeakerStop(uint32_t node_handle);
static int8_t AUDIO_SpeakerChangeFrequency(uint32_t node_handle);
static int8_t AUDIO_SpeakerMute(uint16_t channel_number, uint8_t mute, uint32_t node_handle);
static int8_t AUDIO_SpeakerSetVolume(uint16_t channel_number, int volume, uint32_t node_handle);
static void AUDIO_SpeakerInitInjectionsParams(AUDIO_SpeakerNode_t *speaker);
static int8_t AUDIO_SpeakerStartReadCount(uint32_t node_handle);
static uint16_t AUDIO_SpeakerGetLastReadCount(uint32_t node_handle);

static uint8_t speaker_specific_alt_buffer[SPEAKER_ALT_BUFFER_SIZE] = {0};

/* Private typedef -----------------------------------------------------------*/
#ifdef DEBUG_SPEAKER_NODE
typedef struct
{
    uint32_t time;
    uint16_t injection_size;
    uint16_t read;
    uint16_t dump;
    uint8_t *data;
} AUDIO_SpeakerNodeBufferStats_t;
#endif /* DEBUG_SPEAKER_NODE*/

/* Private macros ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
#ifdef DEBUG_SPEAKER_NODE
extern __IO uint32_t uwTick;
#endif /* DEBUG_SPEAKER_NODE*/

/* Private variables -----------------------------------------------------------*/
static AUDIO_SpeakerNode_t *AUDIO_SpeakerHandler = 0;
#ifdef DEBUG_SPEAKER_NODE
static AUDIO_SpeakerNodeBufferStats_t AUDIO_SpeakerDebugStats[SPEAKER_DEBUG_BUFFER_SIZE];
static int AUDIO_SpeakerDebugStats_count = 0;
#endif /* DEBUG_SPEAKER_NODE*/

/* Exported functions ---------------------------------------------------------*/

/**
 * @brief  AUDIO_SpeakerInit
 *         Initializes the audio speaker node, set callbacks and start the codec. As no data are ready. The
 *         SAI is feeded from the alternate buffer (filled by zeros)
 * @param  audio_description(IN): audio information
 * @param  session_handle(IN):   session handle
 * @param  node_handle(IN):      speaker node handle must be allocated
 * @retval 0 if no error
 */
int8_t AUDIO_SpeakerInit(AUDIO_Description_t *audio_description, AUDIO_Session_t *session_handle,
                         uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker;

    speaker = (AUDIO_SpeakerNode_t *)node_handle;
    memset(speaker, 0, sizeof(AUDIO_SpeakerNode_t));
    speaker->node.type              = AUDIO_OUTPUT;
    speaker->node.state             = AUDIO_NODE_INITIALIZED;
    speaker->node.session_handle    = session_handle;
    speaker->node.audio_description = audio_description;
    speaker->specific.alt_buffer    = speaker_specific_alt_buffer;
    if (speaker->specific.alt_buffer == 0)
    {
        Error_Handler();
    }
    AUDIO_SpeakerInitInjectionsParams(speaker);

    /* set callbacks */
    speaker->SpeakerDeInit          = AUDIO_SpeakerDeInit;
    speaker->SpeakerStart           = AUDIO_SpeakerStart;
    speaker->SpeakerStop            = AUDIO_SpeakerStop;
    speaker->SpeakerChangeFrequency = AUDIO_SpeakerChangeFrequency;
    speaker->SpeakerMute            = AUDIO_SpeakerMute;
    speaker->SpeakerSetVolume       = AUDIO_SpeakerSetVolume;
    speaker->SpeakerStartReadCount  = AUDIO_SpeakerStartReadCount;
    speaker->SpeakerGetReadCount    = AUDIO_SpeakerGetLastReadCount;

    uint16_t ms_package_size = AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(speaker->node.audio_description);
    audio_tx_update_package_size(ms_package_size);
    AUDIO_SpeakerHandler = speaker;
    return 0;
}

// static uint8_t debug_tx_msg = 0;
/**
 * @brief  AUDIO_SpeakerUpdateBuffer
 *         read a packet from the buffer.
 * @param  None
 * @retval None
 */
static uint16_t AUDIO_SpeakerUpdateBuffer(void)
{
    uint16_t wr_distance, read_length = 0;

    if ((AUDIO_SpeakerHandler) && (AUDIO_SpeakerHandler->node.state != AUDIO_NODE_OFF))
    {
        /* if speaker was started prepare next data */
        if (AUDIO_SpeakerHandler->node.state == AUDIO_NODE_STARTED)
        {
            if (AUDIO_SpeakerHandler->specific.cmd & SPEAKER_CMD_STOP)
            {
                AUDIO_SpeakerHandler->specific.data      = AUDIO_SpeakerHandler->specific.alt_buffer;
                AUDIO_SpeakerHandler->specific.data_size = AUDIO_SpeakerHandler->specific.injection_size;
                AUDIO_SpeakerHandler->specific.offset    = 0;
                memset(AUDIO_SpeakerHandler->specific.data, 0, AUDIO_SpeakerHandler->specific.data_size);
                AUDIO_SpeakerHandler->node.state = AUDIO_NODE_STOPPED;
                audio_tx_buffer_reset();
#if USE_I2S_AUDIO_PLAYBACK
                audio_i2s_update_buffer(AUDIO_SpeakerHandler->specific.data, AUDIO_SpeakerHandler->specific.data_size);
#endif
                AUDIO_SpeakerHandler->specific.cmd       ^= SPEAKER_CMD_STOP;
            }
            else
            {
#if USE_I2S_AUDIO_PLAYBACK
                audio_i2s_update_buffer(AUDIO_SpeakerHandler->specific.data, AUDIO_SpeakerHandler->specific.data_size);
#endif
                /* inform session that a packet is played */
                AUDIO_SpeakerHandler->node.session_handle->SessionCallback(AUDIO_PACKET_PLAYED, (AUDIO_Node_t *)AUDIO_SpeakerHandler,
                                                                           AUDIO_SpeakerHandler->node.session_handle);
                AUDIO_SpeakerHandler->specific.data_size = AUDIO_SpeakerHandler->packet_length;
                /* prepare next size to inject */
                read_length = AUDIO_SpeakerHandler->packet_length;

                wr_distance = AUDIO_BUFFER_FILLED_SIZE(AUDIO_SpeakerHandler->buf);
                if (wr_distance < read_length)
                {
                    /** inform session that an underrun is happened */
                    AUDIO_SpeakerHandler->node.session_handle->SessionCallback(AUDIO_UNDERRUN, (AUDIO_Node_t *)AUDIO_SpeakerHandler,
                                                                               AUDIO_SpeakerHandler->node.session_handle);
                    read_length = 0;
                }
                else
                {
                    // Debug UWB transmission data curruption
                    // memset(AUDIO_SpeakerHandler->specific.data, debug_tx_msg++, AUDIO_SpeakerHandler->specific.data_size);
                    /* inject current data */
                    if (0 != audio_tx_task_fill_data(AUDIO_SpeakerHandler->specific.data, AUDIO_SpeakerHandler->specific.data_size))
                    {
                        /** inform session that an overrun is happened */
                        AUDIO_SpeakerHandler->node.session_handle->SessionCallback(AUDIO_OVERRUN, (AUDIO_Node_t *)AUDIO_SpeakerHandler,
                                                                                   AUDIO_SpeakerHandler->node.session_handle);
                        return 0;
                    }
                    /* update read pointer */
                    AUDIO_SpeakerHandler->specific.data = AUDIO_SpeakerHandler->buf->data + AUDIO_SpeakerHandler->buf->rd_ptr;
                    AUDIO_SpeakerHandler->buf->rd_ptr += read_length;
                    if (AUDIO_SpeakerHandler->buf->rd_ptr >= AUDIO_SpeakerHandler->buf->size)
                    {
                        AUDIO_SpeakerHandler->buf->rd_ptr = AUDIO_SpeakerHandler->buf->rd_ptr - AUDIO_SpeakerHandler->buf->size;
                    }
                }
            }
        } /* AUDIO_SpeakerHandler->node.state == AUDIO_NODE_STARTED */
        else
        {
#if USE_I2S_AUDIO_PLAYBACK
            audio_i2s_pause();
#endif
        }
    }

    return read_length;
}

#if USE_I2S_AUDIO_PLAYBACK
/**
 * @brief  AUDIO_SpeakerUpdateBuffer
 *         read a packet from the buffer.
 * @param  None
 * @retval None
 */
static uint16_t AUDIO_I2SSpeakerPlay(void)
{
    uint16_t wr_distance, read_length = 0;
    audio_i2s_start(AUDIO_SpeakerHandler->specific.data, AUDIO_SpeakerHandler->specific.data_size);
    /* inform session that a packet is played */
    AUDIO_SpeakerHandler->node.session_handle->SessionCallback(AUDIO_PACKET_PLAYED, (AUDIO_Node_t *)AUDIO_SpeakerHandler,
                                                               AUDIO_SpeakerHandler->node.session_handle);
    AUDIO_SpeakerHandler->specific.data_size = AUDIO_SpeakerHandler->packet_length;
    /* prepare next size to inject */
    read_length = AUDIO_SpeakerHandler->packet_length;

    wr_distance = AUDIO_BUFFER_FILLED_SIZE(AUDIO_SpeakerHandler->buf);
    if (wr_distance < read_length)
    {
        /** inform session that an underrun is happened */
        AUDIO_SpeakerHandler->node.session_handle->SessionCallback(AUDIO_UNDERRUN, (AUDIO_Node_t *)AUDIO_SpeakerHandler,
                                                                   AUDIO_SpeakerHandler->node.session_handle);
        read_length = 0;
    }
    else
    {
        /* inject current data */
        if (0 != audio_tx_task_fill_data(AUDIO_SpeakerHandler->specific.data, AUDIO_SpeakerHandler->specific.data_size))
        {
            /** inform session that an overrun is happened */
            AUDIO_SpeakerHandler->node.session_handle->SessionCallback(AUDIO_OVERRUN, (AUDIO_Node_t *)AUDIO_SpeakerHandler,
                                                                       AUDIO_SpeakerHandler->node.session_handle);
            return 0;
        }
        /* update read pointer */
        AUDIO_SpeakerHandler->specific.data = AUDIO_SpeakerHandler->buf->data + AUDIO_SpeakerHandler->buf->rd_ptr;
        AUDIO_SpeakerHandler->buf->rd_ptr += read_length;
        if (AUDIO_SpeakerHandler->buf->rd_ptr >= AUDIO_SpeakerHandler->buf->size)
        {
            AUDIO_SpeakerHandler->buf->rd_ptr = AUDIO_SpeakerHandler->buf->rd_ptr - AUDIO_SpeakerHandler->buf->size;
        }
    }
    return read_length;
}
#endif

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  AUDIO_SpeakerDeInit
 *         De-Initializes the audio speaker node
 * @param  node_handle(IN): speaker node handle must be initialized
 * @retval  : 0 if no error
 */
static int8_t AUDIO_SpeakerDeInit(uint32_t node_handle)
{
    /* @TODO implement function */
    AUDIO_SpeakerNode_t *speaker;

    speaker = (AUDIO_SpeakerNode_t *)node_handle;
    if (speaker->node.state != AUDIO_NODE_OFF)
    {
        // audio_tx_terminate();
        // free(speaker->specific.alt_buffer);
        speaker->node.state = AUDIO_NODE_OFF;
    }
    AUDIO_SpeakerHandler = 0;
    return 0;
}

/**
 * @brief  AUDIO_SpeakerStart
 *         Start the audio speaker node
 * @param  buffer(IN):     buffer to use while node is being started
 * @param  node_handle(IN): speaker node handle must be initialized
 * @retval 0 if no error
 */
static int8_t AUDIO_SpeakerStart(AUDIO_CircularBuffer_t *buffer, uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker;

    speaker               = (AUDIO_SpeakerNode_t *)node_handle;
    speaker->buf          = buffer;
    speaker->specific.cmd = 0;
    AUDIO_SpeakerMute(0, speaker->node.audio_description->audio_mute, node_handle);
    AUDIO_SpeakerSetVolume(0, speaker->node.audio_description->audio_volume_db_256, node_handle);
    speaker->node.state = AUDIO_NODE_STARTED;
#if USE_I2S_AUDIO_PLAYBACK
    AUDIO_I2SSpeakerPlay();
#endif
    return 0;
}

/**
 * @brief  AUDIO_SpeakerStop
 *         Stop speaker node. the speaker will be stopped after finalizing current packet transfer.
 * @param  node_handle(IN): speaker node handle must be Started
 * @retval 0 if no error
 */
static int8_t AUDIO_SpeakerStop(uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker;

    speaker                     = (AUDIO_SpeakerNode_t *)node_handle;

    speaker->specific.cmd |= SPEAKER_CMD_STOP;
    return 0;
}

/**
 * @brief  AUDIO_SpeakerChangeFrequency
 *         change frequency then stop speaker node
 * @param  node_handle: speaker node handle must be Started
 * @retval 0 if no error
 */
static int8_t AUDIO_SpeakerChangeFrequency(uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker;

    speaker = (AUDIO_SpeakerNode_t *)node_handle;

    AUDIO_SpeakerInitInjectionsParams(speaker);
    uint16_t ms_package_size = AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(speaker->node.audio_description);
    audio_tx_update_package_size(ms_package_size);
    speaker->node.state = AUDIO_NODE_STOPPED;
    return 0;
}

/**
 * @brief  AUDIO_SpeakerInitInjectionsParams
 *         Stop speaker node
 * @param  speaker(IN): speaker node handle must be Started
 * @retval 0 if no error
 */
static void AUDIO_SpeakerInitInjectionsParams(AUDIO_SpeakerNode_t *speaker)
{
    speaker->packet_length           = AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(speaker->node.audio_description);
    speaker->specific.injection_size = AUDIO_SPEAKER_INJECTION_LENGTH(speaker->node.audio_description);
    speaker->specific.double_buff    = 0;
    speaker->specific.offset         = 0;

    /* update alternative buffer */
    memset(speaker->specific.alt_buffer, 0, SPEAKER_ALT_BUFFER_SIZE);
    speaker->specific.data      = speaker->specific.alt_buffer; /* start injection of dumped data */
    speaker->specific.data_size = speaker->packet_length;
}
/**
 * @brief  AUDIO_SpeakerMute
 *         set Mute value to speaker
 * @param  channel_number(IN): channel number
 * @param  mute(IN): mute value (0 : mute , 1 unmute)
 * @param  node_handle(IN): speaker node handle must be Started
 * @retval  : 0 if no error
 */
static int8_t AUDIO_SpeakerMute(uint16_t channel_number, uint8_t mute, uint32_t node_handle)
{
#if USE_AUDIO_TIMER_VOLUME_CTRL
    AUDIO_SpeakerNode_t *speaker;

    speaker                                     = (AUDIO_SpeakerNode_t *)node_handle;
    speaker->node.audio_description->audio_mute = mute;
    speaker->specific.cmd                       = (speaker->specific.cmd & (~SPEAKER_CMD_MUTE_FIRST)) | SPEAKER_CMD_MUTE_UNMUTE;

#else
    // BSP_AUDIO_OUT_SetMute(mute);
#endif /* USE_AUDIO_TIMER_VOLUME_CTRL */

    return 0;
}

/**
 * @brief  AUDIO_SpeakerSetVolume
 *         set Volume value to speaker
 * @param  channel_number(IN): channel number
 * @param  volume_db_256(IN):  volume value in db
 * @param  node_handle(IN):    speaker node handle must be Started
 * @retval 0 if no error
 */
static int8_t AUDIO_SpeakerSetVolume(uint16_t channel_number, int volume_db_256, uint32_t node_handle)
{
#if USE_AUDIO_TIMER_VOLUME_CTRL
    AUDIO_SpeakerNode_t *speaker;

    speaker                                              = (AUDIO_SpeakerNode_t *)node_handle;
    speaker->node.audio_description->audio_volume_db_256 = volume_db_256;
    speaker->specific.cmd |= (SPEAKER_CMD_MUTE_FIRST | SPEAKER_CMD_CHANGE_VOLUME);
#else
    // BSP_AUDIO_OUT_SetVolume(VOLUME_DB_256_TO_PERCENT(volume_db_256));
#endif /* USE_AUDIO_TIMER_VOLUME_CTRL */
    return 0;
}

#if USE_AUDIO_TIMER_VOLUME_CTRL
/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((AUDIO_SpeakerHandler) && (AUDIO_SpeakerHandler->node.state != AUDIO_NODE_OFF))
    {
        /* Handle volume Commands here */
        if (AUDIO_SpeakerHandler->specific.cmd & SPEAKER_CMD_CHANGE_VOLUME)
        {
            if ((AUDIO_SpeakerHandler->specific.cmd & SPEAKER_CMD_MUTE_FIRST) && (AUDIO_SpeakerHandler->specific.cmd & SPEAKER_CMD_MUTE_UNMUTE))
            {
                // BSP_AUDIO_OUT_SetMute(AUDIO_SpeakerHandler->node.audio_description->audio_mute);
                AUDIO_SpeakerHandler->specific.cmd &= ~SPEAKER_CMD_MUTE_UNMUTE;
            }
            BSP_AUDIO_OUT_SetVolume(VOLUME_DB_256_TO_PERCENT(AUDIO_SpeakerHandler->node.audio_description->audio_volume_db_256));
            AUDIO_SpeakerHandler->specific.cmd &= ~SPEAKER_CMD_CHANGE_VOLUME;
        }
        if (AUDIO_SpeakerHandler->specific.cmd & SPEAKER_CMD_MUTE_UNMUTE)
        {
            BSP_AUDIO_OUT_SetMute(AUDIO_SpeakerHandler->node.audio_description->audio_mute);
            AUDIO_SpeakerHandler->specific.cmd &= ~SPEAKER_CMD_MUTE_UNMUTE;
        }
    }
}
#endif /* USE_AUDIO_TIMER_VOLUME_CTRL */

#if USE_I2S_AUDIO_PLAYBACK
void AUDIO_SpeakerI2SComplete()
{
    AUDIO_SpeakerUpdateBuffer();
}

void AUDIO_SpeakerI2SHalfComplete()
{
    AUDIO_SpeakerUpdateBuffer();
}
/**
 * @brief  AUDIO_SpeakerStartReadCount
 *         Start a counter of how much of byte has been read from the buffer(transmitted to SAI)
 * @param  node_handle: mic node handle must be started
 * @retval  : 0 if no error
 */
static int8_t AUDIO_SpeakerStartReadCount(uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker;

    speaker                         = (AUDIO_SpeakerNode_t *)node_handle;
    I2S_HandleTypeDef *hi2stx       = audio_i2s_get_handle();
    speaker->specific.dma_remaining = __HAL_DMA_GET_COUNTER(hi2stx->hdmatx);
    return 0;
}

/**
 * @brief  AUDIO_SpeakerGetLastReadCount
 *         return the number of bytes have been read and reset the counter
 * @param  node_handle: speaker node handle must be started
 * @retval  :  number of read bytes , 0 if  an error
 */
static uint16_t AUDIO_SpeakerGetLastReadCount(uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker;
    int cur_waiting_bytes, read_bytes, last_packet_size;

    speaker                   = (AUDIO_SpeakerNode_t *)node_handle;
    I2S_HandleTypeDef *hi2stx = audio_i2s_get_handle();

    /* read remind value in dma buffer */
    cur_waiting_bytes = __HAL_DMA_GET_COUNTER(hi2stx->hdmatx);
    last_packet_size = hi2stx->TxXferSize;
    read_bytes       = (speaker->specific.dma_remaining >= cur_waiting_bytes) ? speaker->specific.dma_remaining - cur_waiting_bytes : (last_packet_size - cur_waiting_bytes) + speaker->specific.dma_remaining;


    speaker->specific.dma_remaining = cur_waiting_bytes;


    return read_bytes;
}

#else
/**
 * @brief  AUDIO_SpeakerStartReadCount
 *         Start a counter of how much of byte has been read from the buffer(transmitted to SAI)
 * @param  node_handle: mic node handle must be started
 * @retval  : 0 if no error
 */
static int8_t AUDIO_SpeakerStartReadCount(uint32_t node_handle)
{
    return 0;
}

/**
 * @brief  AUDIO_SpeakerGetLastReadCount
 *         return the number of bytes have been read and reset the counter
 * @param  node_handle: speaker node handle must be started
 * @retval  :  number of read bytes , 0 if  an error
 */
static uint16_t AUDIO_SpeakerGetLastReadCount(uint32_t node_handle)
{
    AUDIO_SpeakerNode_t *speaker = (AUDIO_SpeakerNode_t *)node_handle;
    int read_bytes = AUDIO_SpeakerUpdateBuffer();

    read_bytes = read_bytes / (speaker->node.audio_description->resolution);
#if USB_AUDIO_CONFIG_PLAY_RES_BIT == 24 || USB_AUDIO_CONFIG_PLAY_RES_BIT == 32
    read_bytes = read_bytes * 2;
#endif

#if USE_FEEDBACK_AUDIO_SYNC
    int8_t offset = audio_tx_get_feedback_offset();
    read_bytes = read_bytes + offset;
#endif
    return read_bytes;
}
#endif

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
