#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "stm32f4xx_hal.h"

#define RC_SW_UP   1U
#define RC_SW_DOWN 2U
#define RC_SW_MID  3U

#define RC_S_RIGHT 0U
#define RC_S_LEFT  1U

#define RC_SOURCE_NONE    0U
#define RC_SOURCE_DIRECT  1U
#define RC_SOURCE_FORWARD 2U

typedef struct
{
    int16_t channel[5];
    uint8_t switch_position[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
    uint32_t update_tick_ms;
} remote_control_t;

void Remote_Control_Init(void);
void Remote_Control_UART_IRQHandler(void);
void Remote_Control_Link_UART_IRQHandler(void);
void Remote_Control_Get(remote_control_t *remote);
uint8_t Remote_Control_IsOnline(uint32_t now_ms, uint32_t timeout_ms);
uint8_t Remote_Control_DataValid(const remote_control_t *remote);
uint8_t Remote_Control_GetSource(void);

#endif
