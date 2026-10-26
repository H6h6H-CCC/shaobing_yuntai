#include "dji_motor.h"

#define DJI_CHASSIS_COMMAND_ID 0x200U
#define DJI_GIMBAL_COMMAND_ID  0x1FFU
#define DJI_MOTOR1_FEEDBACK_ID 0x201U
#define DJI_MOTOR4_FEEDBACK_ID 0x204U
#define DJI_6020_FEEDBACK_ID1  0x205U
#define DJI_6020_FEEDBACK_ID3  0x207U

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static volatile dji_motor_feedback_t g_motor[DJI_CHASSIS_MOTOR_COUNT];
static volatile dji_6020_feedback_t g_6020[DJI_GIMBAL_MOTOR_COUNT];

static void DJI_Motor_Parse(uint8_t index, const uint8_t data[8])
{
    volatile dji_motor_feedback_t *motor = &g_motor[index];

    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)((data[0] << 8) | data[1]);
    motor->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor->given_current = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];
    motor->error_code = data[7];
    motor->update_tick_ms = HAL_GetTick();
}

static void DJI_6020_Parse(uint8_t index, const uint8_t data[8])
{
    volatile dji_6020_feedback_t *motor = &g_6020[index];

    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)((data[0] << 8) | data[1]);
    motor->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor->given_current = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];
    motor->reserved_byte = data[7];
    motor->update_tick_ms = HAL_GetTick();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (((hcan != &hcan1) && (hcan != &hcan2)) ||
        (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK))
    {
        return;
    }

    /* 云台板分总线：CAN1只解析3508，CAN2只解析6020。 */
    if ((hcan == &hcan1) && (header.IDE == CAN_ID_STD) &&
        (header.StdId >= DJI_MOTOR1_FEEDBACK_ID) &&
        (header.StdId <= DJI_MOTOR4_FEEDBACK_ID))
    {
        DJI_Motor_Parse((uint8_t)(header.StdId - DJI_MOTOR1_FEEDBACK_ID), data);
    }
    else if ((hcan == &hcan2) && (header.IDE == CAN_ID_STD) &&
             (header.StdId >= DJI_6020_FEEDBACK_ID1) &&
             (header.StdId <= DJI_6020_FEEDBACK_ID3))
    {
        DJI_6020_Parse((uint8_t)(header.StdId - DJI_6020_FEEDBACK_ID1), data);
    }
}

HAL_StatusTypeDef DJI_Motor_SendChassisCurrent(CAN_HandleTypeDef *hcan,
                                               int16_t motor1,
                                               int16_t motor2,
                                               int16_t motor3,
                                               int16_t motor4)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t data[8];
    const int16_t current[4] = {motor1, motor2, motor3, motor4};
    uint8_t i;

    header.StdId = DJI_CHASSIS_COMMAND_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    for (i = 0; i < 4; i++)
    {
        data[i * 2] = (uint8_t)(current[i] >> 8);
        data[i * 2 + 1] = (uint8_t)current[i];
    }

    /* 邮箱满时直接返回忙，避免 HAL 把“暂时无空邮箱”记录成参数错误。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef DJI_6020_SendCurrent(CAN_HandleTypeDef *hcan,
                                       int16_t motor1,
                                       int16_t motor2,
                                       int16_t motor3,
                                       int16_t motor4)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t data[8];
    const int16_t current[4] = {motor1, motor2, motor3, motor4};
    uint8_t i;

    header.StdId = DJI_GIMBAL_COMMAND_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    for (i = 0U; i < 4U; i++)
    {
        data[i * 2U] = (uint8_t)(current[i] >> 8);
        data[i * 2U + 1U] = (uint8_t)current[i];
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

void DJI_Motor_GetFeedback(uint8_t index, dji_motor_feedback_t *feedback)
{
    uint32_t primask;

    if ((index >= DJI_CHASSIS_MOTOR_COUNT) || (feedback == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = g_motor[index];
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t DJI_Motor_IsOnline(uint8_t index, uint32_t now_ms, uint32_t timeout_ms)
{
    dji_motor_feedback_t feedback = {0};

    DJI_Motor_GetFeedback(index, &feedback);
    return (uint8_t)((feedback.update_tick_ms != 0U) &&
                     ((uint32_t)(now_ms - feedback.update_tick_ms) <= timeout_ms));
}

void DJI_6020_GetFeedback(uint8_t index, dji_6020_feedback_t *feedback)
{
    uint32_t primask;

    if ((index >= DJI_GIMBAL_MOTOR_COUNT) || (feedback == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = g_6020[index];
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t DJI_6020_IsOnline(uint8_t index, uint32_t now_ms, uint32_t timeout_ms)
{
    dji_6020_feedback_t feedback = {0};

    DJI_6020_GetFeedback(index, &feedback);
    return (uint8_t)((feedback.update_tick_ms != 0U) &&
                     ((uint32_t)(now_ms - feedback.update_tick_ms) <= timeout_ms));
}
