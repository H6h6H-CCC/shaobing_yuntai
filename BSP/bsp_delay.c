#include "bsp_delay.h"
#include "stm32f4xx_hal.h"

void BSP_DelayUs(uint32_t us)
{
    uint32_t cycles;

    /* 裸机微秒延时，供尚未启用的 BMI088 和 IST8310 驱动调用。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;

    cycles = (SystemCoreClock / 1000000U) * us;
    while (DWT->CYCCNT < cycles)
    {
    }
}
