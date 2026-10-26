#include "shoot_test.h"

#include <stdio.h>

#include "bsp_can.h"
#include "can.h"
#include "dji_motor.h"
#include "pid.h"
#include "pitch_test.h"
#include "remote_control.h"
#include "usart.h"

#define SHOOT_MOTOR_COUNT       3U
#define SHOOT_MOTOR_TIMEOUT_MS  100U
#define SHOOT_REMOTE_TIMEOUT_MS 200U

/* CAN1 0x200: ID1不用，ID2左摩擦轮，ID3右摩擦轮，ID4拨弹轮。 */
/* 实车确认：ID4 拨弹轮需要反向旋转。 */
#define TRIGGER_TARGET_RPM     (-750)
#define LEFT_FRIC_TARGET_RPM   (-5000)
#define RIGHT_FRIC_TARGET_RPM   5000

/* 控制数组顺序固定为：拨弹轮ID4、左摩擦轮ID2、右摩擦轮ID3。 */
static const uint8_t g_feedback_index[SHOOT_MOTOR_COUNT] = {3U, 1U, 2U};

typedef enum
{
    SHOOT_MODE_STOP = 0,
    SHOOT_MODE_TRIGGER,
    SHOOT_MODE_FIRE
} shoot_mode_t;

static pid_t g_speed_pid[SHOOT_MOTOR_COUNT];
static dji_motor_feedback_t g_feedback[SHOOT_MOTOR_COUNT];
static int16_t g_target_rpm[SHOOT_MOTOR_COUNT];
static int16_t g_output_current[SHOOT_MOTOR_COUNT];
static shoot_mode_t g_mode;
static uint8_t g_online_mask;
static uint8_t g_remote_online;
static uint8_t g_remote_valid;
static uint8_t g_output_enabled;
static uint32_t g_can_tx_fail_count;

static void Shoot_Test_ClearControl(void)
{
    uint8_t i;

    for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
    {
        g_target_rpm[i] = 0;
        g_output_current[i] = 0;
        PID_Clear(&g_speed_pid[i]);
    }
}

static const char *Shoot_Test_ModeName(shoot_mode_t mode)
{
    if (mode == SHOOT_MODE_TRIGGER)
    {
        return "TRIGGER";
    }
    if (mode == SHOOT_MODE_FIRE)
    {
        return "FIRE";
    }
    return "STOP";
}

static const char *Shoot_Test_RemoteSourceName(uint8_t source)
{
    if (source == RC_SOURCE_DIRECT)
    {
        return "DIRECT";
    }
    if (source == RC_SOURCE_FORWARD)
    {
        return "FORWARD";
    }
    return "NONE";
}

void Shoot_Test_Init(void)
{
    if (BSP_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 3508走CAN1；启动CAN2，供main中的Pitch_Test控制6020。 */
    if (BSP_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    Remote_Control_Init();

    /* 参数沿用旧工程：拨弹轮限流8000，两个摩擦轮限流16000。 */
    PID_Init(&g_speed_pid[0], 5.0f, 0.0f, 0.0f, 1000.0f, 8000.0f);
    PID_Init(&g_speed_pid[1], 2.5f, 0.1f, 0.0f, 1000.0f, 16000.0f);
    PID_Init(&g_speed_pid[2], 2.5f, 0.1f, 0.0f, 1000.0f, 16000.0f);

    Shoot_Test_ClearControl();
    (void)DJI_Motor_SendChassisCurrent(&hcan1, 0, 0, 0, 0);
    (void)DJI_6020_SendCurrent(&hcan2, 0, 0, 0, 0);
}

void Shoot_Test_Update(void)
{
    remote_control_t remote = {0};
    uint32_t now_ms = HAL_GetTick();
    uint8_t i;

    Remote_Control_Get(&remote);
    g_remote_online = Remote_Control_IsOnline(now_ms, SHOOT_REMOTE_TIMEOUT_MS);
    g_remote_valid = Remote_Control_DataValid(&remote);

    g_online_mask = 0U;
    for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
    {
        DJI_Motor_GetFeedback(g_feedback_index[i], &g_feedback[i]);
        if (DJI_Motor_IsOnline(g_feedback_index[i], now_ms,
                               SHOOT_MOTOR_TIMEOUT_MS) != 0U)
        {
            g_online_mask |= (uint8_t)(1U << g_feedback_index[i]);
        }
    }

    /* 左开关中档是整车控制总使能；右开关中档只运行ID4拨弹轮。 */
    g_mode = SHOOT_MODE_STOP;
    if ((g_remote_online != 0U) && (g_remote_valid != 0U) &&
        (remote.switch_position[RC_S_LEFT] == RC_SW_MID))
    {
        if (remote.switch_position[RC_S_RIGHT] == RC_SW_MID)
        {
            g_mode = SHOOT_MODE_TRIGGER;
        }
        else if (remote.switch_position[RC_S_RIGHT] == RC_SW_UP)
        {
            g_mode = SHOOT_MODE_FIRE;
        }
    }

    /* 只要求遥控器有效；三台3508分别判断在线，不做总在线门槛。 */
    g_output_enabled = (uint8_t)(g_mode != SHOOT_MODE_STOP);
    if (g_output_enabled == 0U)
    {
        Shoot_Test_ClearControl();
    }
    else
    {
        g_target_rpm[0] = ((g_mode == SHOOT_MODE_TRIGGER) ||
                           (g_mode == SHOOT_MODE_FIRE)) ?
                          TRIGGER_TARGET_RPM : 0;
        g_target_rpm[1] = (g_mode == SHOOT_MODE_FIRE) ?
                          LEFT_FRIC_TARGET_RPM : 0;
        g_target_rpm[2] = (g_mode == SHOOT_MODE_FIRE) ?
                          RIGHT_FRIC_TARGET_RPM : 0;

        for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
        {
            if ((g_target_rpm[i] == 0) ||
                (DJI_Motor_IsOnline(g_feedback_index[i],
                                    now_ms,
                                    SHOOT_MOTOR_TIMEOUT_MS) == 0U))
            {
                g_output_current[i] = 0;
                PID_Clear(&g_speed_pid[i]);
            }
            else
            {
                g_output_current[i] = (int16_t)PID_Update(
                    &g_speed_pid[i],
                    (float)g_target_rpm[i],
                    (float)g_feedback[i].speed_rpm);
            }
        }
    }

    if (DJI_Motor_SendChassisCurrent(&hcan1,
                                     0,
                                     g_output_current[1],
                                     g_output_current[2],
                                     g_output_current[0]) != HAL_OK)
    {
        g_can_tx_fail_count++;
    }
}

void Shoot_Test_DebugPrint(void)
{
    remote_control_t remote = {0};
    dji_6020_feedback_t pitch_feedback = {0};
    dji_6020_feedback_t yaw_feedback = {0};
    uint32_t pitch_angle10;
    uint32_t yaw_angle10_display;
    uint8_t remote_source;
    uint8_t yaw_mode;
    int16_t yaw_channel;
    int16_t yaw_angle10_debug;
    int16_t yaw_target10;
    int16_t yaw_error10;
    int16_t yaw_target_rpm;
    int16_t yaw_speed_rpm;
    int16_t yaw_current;
    static char message[512];
    int length;

    Remote_Control_Get(&remote);
    remote_source = Remote_Control_GetSource();
    /* 两个6020角度合并到同一条DMA输出，避免多次发送相互占用。 */
    DJI_6020_GetFeedback(2U, &pitch_feedback);
    DJI_6020_GetFeedback(0U, &yaw_feedback);
    Pitch_Test_GetYawDebug(&yaw_mode,
                           &yaw_channel,
                           &yaw_angle10_debug,
                           &yaw_target10,
                           &yaw_error10,
                           &yaw_target_rpm,
                           &yaw_speed_rpm,
                           &yaw_current);
    pitch_angle10 = ((uint32_t)pitch_feedback.ecd * 3600U + 4096U) / 8192U;
    yaw_angle10_display = ((uint32_t)yaw_feedback.ecd * 3600U + 4096U) / 8192U;
    length = snprintf(message, sizeof(message),
                      "SHOOT:src=%s mode=%s en=%u rc=%u/%u swL=%u swR=%u on=0x%02X "
                      "tgt[4,2,3]=%d,%d,%d rpm[4,2,3]=%d,%d,%d "
                      "cmd[4,2,3]=%d,%d,%d fbI[4,2,3]=%d,%d,%d "
                      "temp[4,2,3]=%u,%u,%u txfail=%lu "
                      "CAN:free=%lu state=%u err=0x%08lX "
                      "PIT:angle=%lu.%lu "
                      "YAW:mode=%u ch=%d a10=%d t10=%d e10=%d rpmT=%d rpm=%d cmd=%d "
                      "angle=%lu.%lu\r\n",
                       Shoot_Test_RemoteSourceName(remote_source),
                       Shoot_Test_ModeName(g_mode),
                      (unsigned int)g_output_enabled,
                      (unsigned int)g_remote_online,
                      (unsigned int)g_remote_valid,
                      (unsigned int)remote.switch_position[RC_S_LEFT],
                      (unsigned int)remote.switch_position[RC_S_RIGHT],
                      (unsigned int)g_online_mask,
                      (int)g_target_rpm[0],
                      (int)g_target_rpm[1],
                      (int)g_target_rpm[2],
                      (int)g_feedback[0].speed_rpm,
                      (int)g_feedback[1].speed_rpm,
                      (int)g_feedback[2].speed_rpm,
                      (int)g_output_current[0],
                      (int)g_output_current[1],
                      (int)g_output_current[2],
                      (int)g_feedback[0].given_current,
                      (int)g_feedback[1].given_current,
                      (int)g_feedback[2].given_current,
                      (unsigned int)g_feedback[0].temperature,
                      (unsigned int)g_feedback[1].temperature,
                      (unsigned int)g_feedback[2].temperature,
                      (unsigned long)g_can_tx_fail_count,
                      (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
                      (unsigned int)HAL_CAN_GetState(&hcan1),
                      (unsigned long)HAL_CAN_GetError(&hcan1),
                       (unsigned long)(pitch_angle10 / 10U),
                       (unsigned long)(pitch_angle10 % 10U),
                       (unsigned int)yaw_mode,
                       (int)yaw_channel,
                       (int)yaw_angle10_debug,
                      (int)yaw_target10,
                      (int)yaw_error10,
                      (int)yaw_target_rpm,
                      (int)yaw_speed_rpm,
                      (int)yaw_current,
                       (unsigned long)(yaw_angle10_display / 10U),
                       (unsigned long)(yaw_angle10_display % 10U));
    if ((length > 0) && (length < (int)sizeof(message)))
    {
        (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                    (uint16_t)length);
    }
}
