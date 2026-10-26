#include "remote_control.h"
#include "bsp_remote.h"

#include <string.h>

#define RC_FRAME_LENGTH                 18U
#define RC_DMA_BUFFER_LENGTH            36U
#define RC_CHANNEL_OFFSET               1024
#define RC_CHANNEL_MAX_ABS              700

/* 旧工程USART1遥控转发协议：0xA6 + 18字节DBUS + 8位累加和。 */
#define RC_FORWARD_HEADER               0xA6U
#define RC_FORWARD_FRAME_LENGTH        20U
#define RC_FORWARD_DMA_BUFFER_LENGTH   40U
#define RC_SOURCE_SELECT_TIMEOUT_MS   200U

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t g_direct_rx_buffer[2][RC_DMA_BUFFER_LENGTH];
static uint8_t g_forward_rx_buffer[2][RC_FORWARD_DMA_BUFFER_LENGTH];
static volatile remote_control_t g_direct_remote;
static volatile remote_control_t g_forward_remote;
static volatile uint8_t g_remote_source = RC_SOURCE_NONE;

static uint8_t g_forward_parse_buffer[RC_FORWARD_FRAME_LENGTH];
static uint16_t g_forward_parse_length;

static void Remote_Control_Decode(const uint8_t data[RC_FRAME_LENGTH],
                                  volatile remote_control_t *target)
{
    remote_control_t decoded = {0};

    if ((data == NULL) || (target == NULL))
    {
        return;
    }

    decoded.channel[0] = (int16_t)(((data[0] | (data[1] << 8)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.channel[1] = (int16_t)((((data[1] >> 3) | (data[2] << 5)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.channel[2] = (int16_t)((((data[2] >> 6) | (data[3] << 2) |
                                     (data[4] << 10)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.channel[3] = (int16_t)((((data[4] >> 1) | (data[5] << 7)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.switch_position[0] = (uint8_t)((data[5] >> 4) & 0x03);
    decoded.switch_position[1] = (uint8_t)(((data[5] >> 4) & 0x0C) >> 2);
    decoded.mouse_x = (int16_t)(data[6] | (data[7] << 8));
    decoded.mouse_y = (int16_t)(data[8] | (data[9] << 8));
    decoded.mouse_z = (int16_t)(data[10] | (data[11] << 8));
    decoded.mouse_left = data[12];
    decoded.mouse_right = data[13];
    decoded.keyboard = (uint16_t)(data[14] | (data[15] << 8));
    decoded.channel[4] = (int16_t)((data[16] | (data[17] << 8)) - RC_CHANNEL_OFFSET);
    decoded.update_tick_ms = HAL_GetTick();

    *target = decoded;
}

void Remote_Control_Init(void)
{
    memset((void *)&g_direct_remote, 0, sizeof(g_direct_remote));
    memset((void *)&g_forward_remote, 0, sizeof(g_forward_remote));
    memset(g_forward_parse_buffer, 0, sizeof(g_forward_parse_buffer));
    g_forward_parse_length = 0U;
    g_remote_source = RC_SOURCE_NONE;

    /* 直连接收器：USART3，100000 baud、偶校验DBUS。 */
    BSP_Remote_Init(&huart3, &hdma_usart3_rx,
                    g_direct_rx_buffer[0], g_direct_rx_buffer[1],
                    RC_DMA_BUFFER_LENGTH);

    /* 底盘转发链路：USART1，115200 baud、8N1。 */
    BSP_Remote_Init(&huart1, &hdma_usart1_rx,
                    g_forward_rx_buffer[0], g_forward_rx_buffer[1],
                    RC_FORWARD_DMA_BUFFER_LENGTH);
}

void Remote_Control_UART_IRQHandler(void)
{
    uint16_t received_length;
    uint8_t *completed_buffer;

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == RESET)
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    received_length = (uint16_t)(RC_DMA_BUFFER_LENGTH - hdma_usart3_rx.Instance->NDTR);
    hdma_usart3_rx.Instance->NDTR = RC_DMA_BUFFER_LENGTH;

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == 0U)
    {
        completed_buffer = g_direct_rx_buffer[0];
        SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
    }
    else
    {
        completed_buffer = g_direct_rx_buffer[1];
        CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
    }

    __HAL_DMA_ENABLE(&hdma_usart3_rx);

    if (received_length == RC_FRAME_LENGTH)
    {
        Remote_Control_Decode(completed_buffer, &g_direct_remote);
    }
}

static uint8_t Remote_Control_ForwardFrameValid(
    const uint8_t frame[RC_FORWARD_FRAME_LENGTH])
{
    uint8_t checksum = 0U;
    uint8_t i;

    if ((frame == NULL) || (frame[0] != RC_FORWARD_HEADER))
    {
        return 0U;
    }

    for (i = 0U; i < (RC_FORWARD_FRAME_LENGTH - 1U); i++)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    return (uint8_t)(checksum == frame[RC_FORWARD_FRAME_LENGTH - 1U]);
}

static void Remote_Control_ForwardParse(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    for (i = 0U; i < length; i++)
    {
        uint8_t byte = data[i];

        if (g_forward_parse_length == 0U)
        {
            if (byte == RC_FORWARD_HEADER)
            {
                g_forward_parse_buffer[0] = byte;
                g_forward_parse_length = 1U;
            }
            continue;
        }

        g_forward_parse_buffer[g_forward_parse_length] = byte;
        g_forward_parse_length++;

        if (g_forward_parse_length >= RC_FORWARD_FRAME_LENGTH)
        {
            if (Remote_Control_ForwardFrameValid(g_forward_parse_buffer) != 0U)
            {
                Remote_Control_Decode(&g_forward_parse_buffer[1],
                                      &g_forward_remote);
                g_forward_parse_length = 0U;
            }
            else
            {
                /* 校验失败时保留当前字节作为下一帧帧头候选。 */
                if (byte == RC_FORWARD_HEADER)
                {
                    g_forward_parse_buffer[0] = byte;
                    g_forward_parse_length = 1U;
                }
                else
                {
                    g_forward_parse_length = 0U;
                }
            }
        }
    }
}

void Remote_Control_Link_UART_IRQHandler(void)
{
    uint16_t received_length;
    uint8_t *completed_buffer;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == RESET)
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    received_length = (uint16_t)(RC_FORWARD_DMA_BUFFER_LENGTH - hdma_usart1_rx.Instance->NDTR);
    hdma_usart1_rx.Instance->NDTR = RC_FORWARD_DMA_BUFFER_LENGTH;

    if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == 0U)
    {
        completed_buffer = g_forward_rx_buffer[0];
        SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_CT);
    }
    else
    {
        completed_buffer = g_forward_rx_buffer[1];
        CLEAR_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_CT);
    }

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    Remote_Control_ForwardParse(completed_buffer, received_length);
}

uint8_t Remote_Control_DataValid(const remote_control_t *remote)
{
    uint8_t i;

    if (remote == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < 5U; i++)
    {
        if ((remote->channel[i] > RC_CHANNEL_MAX_ABS) ||
            (remote->channel[i] < -RC_CHANNEL_MAX_ABS))
        {
            return 0U;
        }
    }

    if ((remote->switch_position[0] == 0U) ||
        (remote->switch_position[1] == 0U))
    {
        return 0U;
    }
    return 1U;
}

static uint8_t Remote_Control_Select(remote_control_t *selected)
{
    remote_control_t direct = {0};
    remote_control_t forward = {0};
    uint32_t now_ms = HAL_GetTick();
    uint32_t primask;
    uint8_t direct_online;
    uint8_t forward_online;

    primask = __get_PRIMASK();
    __disable_irq();
    direct = g_direct_remote;
    forward = g_forward_remote;
    if (primask == 0U)
    {
        __enable_irq();
    }

    direct_online = (uint8_t)((direct.update_tick_ms != 0U) &&
                              ((uint32_t)(now_ms - direct.update_tick_ms) <=
                               RC_SOURCE_SELECT_TIMEOUT_MS) &&
                              (Remote_Control_DataValid(&direct) != 0U));
    forward_online = (uint8_t)((forward.update_tick_ms != 0U) &&
                               ((uint32_t)(now_ms - forward.update_tick_ms) <=
                                RC_SOURCE_SELECT_TIMEOUT_MS) &&
                               (Remote_Control_DataValid(&forward) != 0U));

    if (direct_online != 0U)
    {
        *selected = direct;
        g_remote_source = RC_SOURCE_DIRECT;
        return RC_SOURCE_DIRECT;
    }
    if (forward_online != 0U)
    {
        *selected = forward;
        g_remote_source = RC_SOURCE_FORWARD;
        return RC_SOURCE_FORWARD;
    }

    memset(selected, 0, sizeof(*selected));
    g_remote_source = RC_SOURCE_NONE;
    return RC_SOURCE_NONE;
}

void Remote_Control_Get(remote_control_t *remote)
{
    if (remote == NULL)
    {
        return;
    }

    (void)Remote_Control_Select(remote);
}

uint8_t Remote_Control_IsOnline(uint32_t now_ms, uint32_t timeout_ms)
{
    remote_control_t remote = {0};

    Remote_Control_Get(&remote);
    return (uint8_t)((remote.update_tick_ms != 0U) &&
                     ((uint32_t)(now_ms - remote.update_tick_ms) <= timeout_ms));
}

uint8_t Remote_Control_GetSource(void)
{
    remote_control_t remote = {0};

    Remote_Control_Get(&remote);
    return g_remote_source;
}
