#include "stm32f7xx.h"

#define HSE_STARTUP_TIMEOUT     0x50000U
#define PLL_STARTUP_TIMEOUT     0x50000U
#define CLOCK_SWITCH_TIMEOUT    0x50000U

static uint32_t wait_until_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout)
{
    while ((*reg & mask) == 0U)
    {
        if (timeout-- == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static void system_clock_config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    /* 216 MHz requires voltage scale 1. */
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS;

    /* HSE input: 8 MHz crystal, PLL output: 216 MHz. */
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEBYP);
    RCC->CR |= RCC_CR_HSEON;
    if (wait_until_set(&RCC->CR, RCC_CR_HSERDY, HSE_STARTUP_TIMEOUT) == 0U)
    {
        SystemCoreClockUpdate();
        return;
    }

    RCC->PLLCFGR =
        RCC_PLLCFGR_PLLSRC_HSE |
        (8U << RCC_PLLCFGR_PLLM_Pos) |
        (432U << RCC_PLLCFGR_PLLN_Pos) |
        (0U << RCC_PLLCFGR_PLLP_Pos) |
        (9U << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    if (wait_until_set(&RCC->CR, RCC_CR_PLLRDY, PLL_STARTUP_TIMEOUT) == 0U)
    {
        SystemCoreClockUpdate();
        return;
    }

    /* 216 MHz requires seven Flash wait states. */
    FLASH->ACR =
        (FLASH->ACR & ~FLASH_ACR_LATENCY) |
        FLASH_ACR_LATENCY_7WS |
        FLASH_ACR_PRFTEN;

    RCC->CFGR =
        (RCC->CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
        RCC_CFGR_HPRE_DIV1 |
        RCC_CFGR_PPRE1_DIV4 |
        RCC_CFGR_PPRE2_DIV2;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    if (wait_until_set(&RCC->CFGR, RCC_CFGR_SWS_PLL, CLOCK_SWITCH_TIMEOUT) == 0U)
    {
        SystemCoreClockUpdate();
        return;
    }

    SystemCoreClock = 216000000U;
}

int main(void)
{
    system_clock_config();

    for (;;)
    {
        __NOP();
    }
}
