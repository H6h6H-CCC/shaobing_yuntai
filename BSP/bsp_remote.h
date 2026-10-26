#ifndef BSP_REMOTE_H
#define BSP_REMOTE_H

#include "stm32f4xx_hal.h"

void BSP_Remote_Init(UART_HandleTypeDef *huart,
                     DMA_HandleTypeDef *hdma_rx,
                     uint8_t *buffer0,
                     uint8_t *buffer1,
                     uint16_t buffer_size);

#endif
