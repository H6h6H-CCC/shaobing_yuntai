#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "stm32f4xx_hal.h"

#define DJI_CHASSIS_MOTOR_COUNT 4U
#define DJI_GIMBAL_MOTOR_COUNT  3U

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    uint8_t error_code;
    uint32_t update_tick_ms;
} dji_motor_feedback_t;

/* CAN2 GM6020反馈：0x205、0x206、0x207对应ID1、ID2、ID3。 */
typedef struct
{
    uint16_t ecd;              /* 编码器原始角度，范围 0~8191 */
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    uint8_t reserved_byte;
    uint32_t update_tick_ms;
} dji_6020_feedback_t;

/* 云台3508使用CAN1，调用时传入 &hcan1，发送0x200。 */
HAL_StatusTypeDef DJI_Motor_SendChassisCurrent(CAN_HandleTypeDef *hcan,
                                               int16_t motor1,
                                               int16_t motor2,
                                               int16_t motor3,
                                               int16_t motor4);
/* 云台6020使用CAN2，传入 &hcan2，0x1FF槽位为ID1、ID2、ID3、预留。 */
HAL_StatusTypeDef DJI_6020_SendCurrent(CAN_HandleTypeDef *hcan,
                                       int16_t motor1,
                                       int16_t motor2,
                                       int16_t motor3,
                                       int16_t motor4);
void DJI_Motor_GetFeedback(uint8_t index, dji_motor_feedback_t *feedback);
uint8_t DJI_Motor_IsOnline(uint8_t index, uint32_t now_ms, uint32_t timeout_ms);
void DJI_6020_GetFeedback(uint8_t index, dji_6020_feedback_t *feedback);
uint8_t DJI_6020_IsOnline(uint8_t index, uint32_t now_ms, uint32_t timeout_ms);

#endif
