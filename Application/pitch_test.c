#include "pitch_test.h"

#include <stdio.h>

#include "can.h"
#include "dji_motor.h"
#include "pid.h"
#include "remote_control.h"
#include "usart.h"

/* ID1 Yaw：迁移旧工程的“手动跟随 + 松杆位置保持”串级控制。 */
#define YAW_MOTOR_INDEX         0U       /* 6020 ID1，反馈帧 0x205。 */
#define YAW_RC_DEADZONE         10
#define YAW_RC_DIRECTION       -1.0f
#define YAW_RC_ANGLE_GAIN       0.05f
#define YAW_HOLD_DELAY_MS       100U
#define YAW_ANGLE_KP             1.35f  /* 调试：提高角度跟随灵敏度。 */
#define YAW_ANGLE_MAX_RPM        160.0f
#define YAW_SPEED_KP             180.0f
#define YAW_SPEED_KI             0.35f
#define YAW_SPEED_MAX_I          300.0f
#define YAW_CURRENT_LIMIT        16000.0f

#define PITCH_MOTOR_INDEX       2U       /* 6020 ID3，反馈帧 0x207。 */
#define PITCH_MOTOR_TIMEOUT_MS  100U
#define PITCH_REMOTE_TIMEOUT_MS 200U
#define PITCH_RC_DEADZONE       40
#define PITCH_CONTROL_PERIOD_S   0.002f /* main每2ms调用一次控制。 */
#define PITCH_RC_MAX_SPEED_DEG_S 55.0f /* 满杆目标移动速度；按实测手感从45提高到50度/s。 */
#define PITCH_RC_STEP            (PITCH_RC_MAX_SPEED_DEG_S * PITCH_CONTROL_PERIOD_S / 660.0f)
#define PITCH_RC_DIRECTION      -1.0f
#define PITCH_MOTOR_DIRECTION    1.0f
/* CAN2 ID3编码器绝对角：目标软限位26~80度，失能后仍为零输出。 */
#define PITCH_ANGLE_MIN_DEG     26.0f
#define PITCH_ANGLE_MAX_DEG     80.0f

/* 位置PI：P从950小步提高到1000，小积分补偿负载静差，不使用速度内环。 */
#define PITCH_ANGLE_KP          1050.0f
#define PITCH_ANGLE_KI          65.0f /* 按秒积分；2ms一次相当于每次累加0.12倍角度误差。 */
#define PITCH_INTEGRAL_LIMIT    1500.0f /* 积分单独限幅，避免受阻时不断累积。 */
#define PITCH_CURRENT_LIMIT     16000.0f

static dji_6020_feedback_t g_pitch_feedback;
static float g_pitch_target_angle;
static uint8_t g_pitch_target_initialized;
static uint8_t g_pitch_motor_online;
static uint8_t g_pitch_remote_online;
static uint8_t g_pitch_remote_valid;
/* 调参快照：保存本轮真正参与控制的输入、误差和输出。 */
static int16_t g_pitch_debug_channel;
static int16_t g_pitch_debug_current;
static float g_pitch_debug_error;
static float g_pitch_integral;
static uint32_t g_pitch_can_tx_fail;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_iout;
    float max_out;
    float error[3];
    float iout;
    float out;
} yaw_legacy_pid_t;

enum
{
    YAW_MODE_MANUAL = 0U,
    YAW_MODE_HOLD = 1U,
    YAW_MODE_DISABLED = 2U
};

static uint8_t g_yaw_target_initialized;
static uint8_t g_yaw_mode = YAW_MODE_DISABLED;
static uint32_t g_yaw_last_input_ms;
static float g_yaw_hold_angle;
static yaw_legacy_pid_t g_yaw_angle_pid;
static yaw_legacy_pid_t g_yaw_speed_pid;

/* 供串口调试读取的 Yaw 状态，单位：角度为0.1度。 */
static int16_t g_yaw_debug_channel;
static int16_t g_yaw_debug_angle10;
static int16_t g_yaw_debug_target10;
static int16_t g_yaw_debug_error10;
static int16_t g_yaw_debug_target_rpm;
static int16_t g_yaw_debug_speed_rpm;
static int16_t g_yaw_debug_current;

static float Pitch_Test_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static void Yaw_LegacyPid_Init(yaw_legacy_pid_t *pid,
                               float kp,
                               float ki,
                               float kd,
                               float max_iout,
                               float max_out)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_iout = max_iout;
    pid->max_out = max_out;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;
}

static void Yaw_LegacyPid_Clear(yaw_legacy_pid_t *pid)
{
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;
}

static float Yaw_LegacyPid_Position(yaw_legacy_pid_t *pid, float error)
{
    float derivative;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = error;
    pid->iout += pid->ki * pid->error[0];
    pid->iout = Pitch_Test_Clamp(pid->iout, pid->max_iout);
    derivative = pid->error[0] - pid->error[1];
    pid->out = pid->kp * pid->error[0] + pid->iout + pid->kd * derivative;
    return Pitch_Test_Clamp(pid->out, pid->max_out);
}

static float Yaw_LegacyPid_Delta(yaw_legacy_pid_t *pid,
                                 float target,
                                 float feedback)
{
    float error;
    float delta;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    error = target - feedback;
    pid->error[0] = error;
    delta = pid->kp * (pid->error[0] - pid->error[1]) +
            pid->ki * pid->error[0] +
            pid->kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->out = Pitch_Test_Clamp(pid->out + delta, pid->max_out);
    return pid->out;
}

static float Pitch_Test_GetAngle(const dji_6020_feedback_t *feedback)
{
    return ((float)feedback->ecd * 360.0f) / 8192.0f;
}

static float Pitch_Test_ClampAngle(float angle)
{
    if (angle < PITCH_ANGLE_MIN_DEG)
    {
        return PITCH_ANGLE_MIN_DEG;
    }
    if (angle > PITCH_ANGLE_MAX_DEG)
    {
        return PITCH_ANGLE_MAX_DEG;
    }
    return angle;
}

static float Yaw_ShortestError(float target, float current)
{
    float error = target - current;

    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

static int16_t Yaw_Update(const remote_control_t *remote, uint32_t now_ms)
{
    dji_6020_feedback_t feedback = {0};
    float current_angle;
    float angle_error;
    float target_rpm;
    int16_t rc_input;

    /* 两轴各自判断电机在线，ID2和另一轴离线都不会挡住本轴控制。 */
    if ((g_pitch_remote_online == 0U) ||
        (g_pitch_remote_valid == 0U) ||
        (remote->switch_position[RC_S_LEFT] != RC_SW_MID) ||
        (DJI_6020_IsOnline(YAW_MOTOR_INDEX, now_ms,
                            PITCH_MOTOR_TIMEOUT_MS) == 0U))
    {
        g_yaw_target_initialized = 0U;
        g_yaw_mode = YAW_MODE_DISABLED;
        Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
        Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
        g_yaw_debug_channel = 0;
        g_yaw_debug_angle10 = 0;
        g_yaw_debug_target10 = 0;
        g_yaw_debug_error10 = 0;
        g_yaw_debug_target_rpm = 0;
        g_yaw_debug_speed_rpm = 0;
        g_yaw_debug_current = 0;
        return 0;
    }

    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &feedback);
    current_angle = Pitch_Test_GetAngle(&feedback);
    if (g_yaw_target_initialized == 0U)
    {
        /* 重新使能时从当前位置开始，避免追赶上次停用前的目标。 */
        g_yaw_hold_angle = current_angle;
        g_yaw_last_input_ms = now_ms;
        g_yaw_mode = YAW_MODE_MANUAL;
        g_yaw_target_initialized = 1U;
    }

    rc_input = remote->channel[0];
    if ((rc_input >= -YAW_RC_DEADZONE) && (rc_input <= YAW_RC_DEADZONE))
    {
        rc_input = 0;
    }

    if (rc_input != 0)
    {
        /* 旧工程手动模式：摇杆直接生成角度误差，等效为速度控制。 */
        g_yaw_last_input_ms = now_ms;
        g_yaw_mode = YAW_MODE_MANUAL;
        angle_error = (float)rc_input * YAW_RC_ANGLE_GAIN *
                      YAW_RC_DIRECTION;
    }
    else if ((uint32_t)(now_ms - g_yaw_last_input_ms) >= YAW_HOLD_DELAY_MS)
    {
        if (g_yaw_mode != YAW_MODE_HOLD)
        {
            /* 旧工程位置保持模式：进入时锁定当前角度。 */
            g_yaw_hold_angle = current_angle;
            g_yaw_mode = YAW_MODE_HOLD;
            Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
            Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
        }
        angle_error = Yaw_ShortestError(g_yaw_hold_angle, current_angle);
    }
    else
    {
        g_yaw_mode = YAW_MODE_MANUAL;
        angle_error = 0.0f;
    }

    target_rpm = Yaw_LegacyPid_Position(&g_yaw_angle_pid, angle_error);
    g_yaw_debug_channel = remote->channel[0];
    g_yaw_debug_angle10 = (int16_t)(current_angle * 10.0f);
    g_yaw_debug_target10 = (int16_t)((current_angle + angle_error) * 10.0f);
    g_yaw_debug_error10 = (int16_t)(angle_error * 10.0f);
    g_yaw_debug_target_rpm = (int16_t)target_rpm;
    g_yaw_debug_speed_rpm = feedback.speed_rpm;
    g_yaw_debug_current = (int16_t)Yaw_LegacyPid_Delta(
        &g_yaw_speed_pid, target_rpm, (float)feedback.speed_rpm);
    return g_yaw_debug_current;
}

void Pitch_Test_Init(void)
{
    /* main先调用Shoot_Test_Init，共用已经启动的CAN2和遥控器接收。 */
    g_pitch_target_initialized = 0U;
    g_pitch_motor_online = 0U;
    g_pitch_remote_online = 0U;
    g_pitch_remote_valid = 0U;
    g_pitch_debug_channel = 0;
    g_pitch_debug_current = 0;
    g_pitch_debug_error = 0.0f;
    g_pitch_integral = 0.0f;
    g_pitch_can_tx_fail = 0U;
    g_yaw_target_initialized = 0U;
    g_yaw_mode = YAW_MODE_DISABLED;
    Yaw_LegacyPid_Init(&g_yaw_angle_pid, YAW_ANGLE_KP, 0.0f, 0.0f,
                       0.0f, YAW_ANGLE_MAX_RPM);
    Yaw_LegacyPid_Init(&g_yaw_speed_pid, YAW_SPEED_KP, YAW_SPEED_KI, 0.0f,
                       YAW_SPEED_MAX_I, YAW_CURRENT_LIMIT);

    /* 本模块只控制CAN2的6020，CAN1发射由shoot_test统一控制。 */
    (void)DJI_6020_SendCurrent(&hcan2, 0, 0, 0, 0);
}

void Pitch_Test_Update(void)
{
    remote_control_t remote = {0};
    uint32_t now_ms = HAL_GetTick();
    float current_angle;
    float angle_error;
    float current_request;
    int16_t pitch_current = 0;
    int16_t yaw_current;
    int16_t rc_input;

    Remote_Control_Get(&remote);
    g_pitch_remote_online = Remote_Control_IsOnline(now_ms,
                                                    PITCH_REMOTE_TIMEOUT_MS);
    g_pitch_remote_valid = Remote_Control_DataValid(&remote);

    DJI_6020_GetFeedback(PITCH_MOTOR_INDEX, &g_pitch_feedback);
    g_pitch_motor_online = DJI_6020_IsOnline(PITCH_MOTOR_INDEX,
                                             now_ms,
                                             PITCH_MOTOR_TIMEOUT_MS);
    current_angle = Pitch_Test_GetAngle(&g_pitch_feedback);
    g_pitch_debug_channel = remote.channel[1];
    g_pitch_debug_error = 0.0f;

    /* 左开关置中对应旧工程的手动模式，其他状态只发零电流。 */
    if ((g_pitch_motor_online == 0U) ||
        (g_pitch_remote_online == 0U) ||
        (g_pitch_remote_valid == 0U) ||
        (remote.switch_position[RC_S_LEFT] != RC_SW_MID))
    {
        g_pitch_target_initialized = 0U;
        /* 失能清积分，重新启用时不带入上次补偿。 */
        g_pitch_integral = 0.0f;
        /* 失能时显示当前位置；下次使能重新钳位，不追赶旧目标。 */
        g_pitch_target_angle = current_angle;
    }
    else
    {
        if (g_pitch_target_initialized == 0U)
        {
            g_pitch_target_angle = Pitch_Test_ClampAngle(current_angle);
            g_pitch_integral = 0.0f;
            g_pitch_target_initialized = 1U;
        }

        /* 右摇杆垂直通道为 channel[1]，目标角度限制在26~80度。 */
        rc_input = remote.channel[1];
        if ((rc_input < PITCH_RC_DEADZONE) &&
            (rc_input > -PITCH_RC_DEADZONE))
        {
            rc_input = 0;
        }
        g_pitch_target_angle += (float)rc_input * PITCH_RC_STEP *
                                PITCH_RC_DIRECTION;
        g_pitch_target_angle = Pitch_Test_ClampAngle(g_pitch_target_angle);

        angle_error = g_pitch_target_angle - current_angle;
        g_pitch_debug_error = angle_error;
        current_request = angle_error * PITCH_ANGLE_KP + g_pitch_integral;
        /* 输出饱和时仅允许反向积分，避免继续向饱和方向积累。 */
        if (((current_request < PITCH_CURRENT_LIMIT) &&
             (current_request > -PITCH_CURRENT_LIMIT)) ||
            ((current_request >= PITCH_CURRENT_LIMIT) && (angle_error < 0.0f)) ||
            ((current_request <= -PITCH_CURRENT_LIMIT) && (angle_error > 0.0f)))
        {
            g_pitch_integral = Pitch_Test_Clamp(
                g_pitch_integral + angle_error * PITCH_ANGLE_KI * PITCH_CONTROL_PERIOD_S,
                PITCH_INTEGRAL_LIMIT);
        }
        pitch_current = (int16_t)Pitch_Test_Clamp(
            (angle_error * PITCH_ANGLE_KP + g_pitch_integral) * PITCH_MOTOR_DIRECTION,
            PITCH_CURRENT_LIMIT);
    }

    yaw_current = Yaw_Update(&remote, now_ms);
    g_pitch_debug_current = pitch_current;
    /* CAN2统一发送：ID1 Yaw、ID3 Pitch，ID2固定零输出。 */
    if (DJI_6020_SendCurrent(&hcan2, yaw_current, 0, pitch_current, 0) != HAL_OK)
    {
        g_pitch_can_tx_fail++;
    }
}

void Pitch_Test_DebugPrint(void)
{
    static char message[256];
    uint32_t now_ms = HAL_GetTick();
    float angle = Pitch_Test_GetAngle(&g_pitch_feedback);
    int limit = (angle <= PITCH_ANGLE_MIN_DEG) ? -1 :
                ((angle >= PITCH_ANGLE_MAX_DEG) ? 1 : 0);
    int length;

    /* DMA未发完前不改缓冲；a10/t10/e10及kp10都需除10得到实际值。 */
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }
    length = snprintf(message, sizeof(message),
                      "PIT:ms=%lu en=%u rc=%u/%u on=%u ch=%d a10=%d t10=%d "
                      "e10=%d rpm=%d cmd=%d fbI=%d T=%u lim=%d age=%lu "
                      "kp10=%d txfail=%lu err=0x%08lX vmax10=%d aki10=%d iout=%d\r\n",
                      (unsigned long)now_ms,
                      (unsigned int)g_pitch_target_initialized,
                      (unsigned int)g_pitch_remote_online,
                      (unsigned int)g_pitch_remote_valid,
                      (unsigned int)g_pitch_motor_online,
                      (int)g_pitch_debug_channel,
                      (int)(angle * 10.0f),
                      (int)(g_pitch_target_angle * 10.0f),
                      (int)(g_pitch_debug_error * 10.0f),
                      (int)g_pitch_feedback.speed_rpm,
                      (int)g_pitch_debug_current,
                      (int)g_pitch_feedback.given_current,
                      (unsigned int)g_pitch_feedback.temperature,
                      limit,
                      (unsigned long)(now_ms - g_pitch_feedback.update_tick_ms),
                      (int)(PITCH_ANGLE_KP * 10.0f),
                      (unsigned long)g_pitch_can_tx_fail,
                      (unsigned long)HAL_CAN_GetError(&hcan2),
                      (int)(PITCH_RC_MAX_SPEED_DEG_S * 10.0f),
                      (int)(PITCH_ANGLE_KI * 10.0f),
                      (int)g_pitch_integral);
    if ((length > 0) && (length < (int)sizeof(message)))
    {
        (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                    (uint16_t)length);
    }
}

void Pitch_Test_GetYawDebug(uint8_t *mode,
                            int16_t *channel,
                            int16_t *angle10,
                            int16_t *target10,
                            int16_t *error10,
                            int16_t *target_rpm,
                            int16_t *speed_rpm,
                            int16_t *current)
{
    if (mode != 0)
    {
        *mode = g_yaw_mode;
    }
    if (channel != 0)
    {
        *channel = g_yaw_debug_channel;
    }
    if (angle10 != 0)
    {
        *angle10 = g_yaw_debug_angle10;
    }
    if (target10 != 0)
    {
        *target10 = g_yaw_debug_target10;
    }
    if (error10 != 0)
    {
        *error10 = g_yaw_debug_error10;
    }
    if (target_rpm != 0)
    {
        *target_rpm = g_yaw_debug_target_rpm;
    }
    if (speed_rpm != 0)
    {
        *speed_rpm = g_yaw_debug_speed_rpm;
    }
    if (current != 0)
    {
        *current = g_yaw_debug_current;
    }
}
