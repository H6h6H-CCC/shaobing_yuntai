#include "bsp_remote.h"

void BSP_Remote_Init(UART_HandleTypeDef *huart,
                     DMA_HandleTypeDef *hdma_rx,
                     uint8_t *buffer0,
                     uint8_t *buffer1,
                     uint16_t buffer_size)
{
    if ((huart == NULL) || (hdma_rx == NULL) ||
        (buffer0 == NULL) || (buffer1 == NULL))
    {
        return;
    }

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    __HAL_DMA_DISABLE(hdma_rx);
    while ((hdma_rx->Instance->CR & DMA_SxCR_EN) != 0U)
    {
        __HAL_DMA_DISABLE(hdma_rx);
    }

    hdma_rx->Instance->PAR = (uint32_t)&huart->Instance->DR;
    hdma_rx->Instance->M0AR = (uint32_t)buffer0;
    hdma_rx->Instance->M1AR = (uint32_t)buffer1;
    hdma_rx->Instance->NDTR = buffer_size;
    SET_BIT(hdma_rx->Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(hdma_rx);
}
