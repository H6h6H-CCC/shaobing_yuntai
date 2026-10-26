#include "gimbal_base.h"

#include <stdio.h>

#include "bsp_can.h"
#include "can.h"
#include "dji_motor.h"
#include "remote_control.h"
#include "usart.h"

#define GIMBAL_BASE_MOTOR_TIMEOUT_MS 100U

static dji_6020_feedback_t g_feedback[DJI_GIMBAL_MOTOR_COUNT];
static uint8_t g_online_mask;
static uint32_t g_can_tx_fail_count;

void Gimbal_Base_Init(void)
{
    /* 6020基础测试改用CAN2，3508的CAN1不受影响。 */
    if (BSP_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    Remote_Control_Init();
    (void)DJI_6020_SendCurrent(&hcan2, 0, 0, 0, 0);
}

void Gimbal_Base_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t i;

    g_online_mask = 0U;
    for (i = 0U; i < DJI_GIMBAL_MOTOR_COUNT; i++)
    {
        DJI_6020_GetFeedback(i, &g_feedback[i]);
        if ((g_feedback[i].update_tick_ms != 0U) &&
            ((uint32_t)(now_ms - g_feedback[i].update_tick_ms) <=
             GIMBAL_BASE_MOTOR_TIMEOUT_MS))
        {
            g_online_mask |= (uint8_t)(1U << i);
        }
    }

    /* 基础工程默认不驱动云台，先持续发送零电流保证上电安全。 */
    if (DJI_6020_SendCurrent(&hcan2, 0, 0, 0, 0) != HAL_OK)
    {
        g_can_tx_fail_count++;
    }
}

void Gimbal_Base_DebugPrint(void)
{
    remote_control_t remote = {0};
    static char message[320];
    int length;
    uint32_t now_ms = HAL_GetTick();

    Remote_Control_Get(&remote);
    length = snprintf(message, sizeof(message),
                      "GIMBAL:rc=%u/%u sw=%u ch=%d,%d on=0x%02X "
                      "M1=%u,%d,%d,%d M2=%u,%d,%d,%d M3=%u,%d,%d,%d "
                      "txfail=%lu\r\n",
                      (unsigned int)Remote_Control_IsOnline(now_ms, 200U),
                      (unsigned int)Remote_Control_DataValid(&remote),
                      (unsigned int)remote.switch_position[RC_S_LEFT],
                      (int)remote.channel[0], (int)remote.channel[1],
                      (unsigned int)g_online_mask,
                      (unsigned int)g_feedback[0].ecd,
                      (int)g_feedback[0].speed_rpm,
                      (int)g_feedback[0].given_current,
                      (int)g_feedback[0].temperature,
                      (unsigned int)g_feedback[1].ecd,
                      (int)g_feedback[1].speed_rpm,
                      (int)g_feedback[1].given_current,
                      (int)g_feedback[1].temperature,
                      (unsigned int)g_feedback[2].ecd,
                      (int)g_feedback[2].speed_rpm,
                      (int)g_feedback[2].given_current,
                      (int)g_feedback[2].temperature,
                      (unsigned long)g_can_tx_fail_count);
    if ((length > 0) && (length < (int)sizeof(message)))
    {
        (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                    (uint16_t)length);
    }
}
