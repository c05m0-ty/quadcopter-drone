#include "stm32f7xx_hal.h"
#include "usb_cdc.h"

#define LED_PIN 15U

extern volatile uint32_t usb_cdc_init_failure;

static uint32_t system_clock_config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};
    RCC_PeriphCLKInitTypeDef peripheral = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        return 0U;
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.HSIState = RCC_HSI_OFF;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 8U;
    oscillator.PLL.PLLN = 432U;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 9U;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        return 0U;
    }

    peripheral.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    peripheral.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
    peripheral.PLLSAI.PLLSAIN = 384U;
    peripheral.PLLSAI.PLLSAIQ = 7U;
    peripheral.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral) != HAL_OK)
    {
        return 0U;
    }

    clocks.ClockType =
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV4;
    clocks.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_7) != HAL_OK)
    {
        return 0U;
    }

    (void)HAL_InitTick(TICK_INT_PRIORITY);
    return 1U;
}

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    GPIOA->MODER =
        (GPIOA->MODER & ~GPIO_MODER_MODER15_Msk) |
        GPIO_MODER_MODER15_0;

    GPIOA->BSRR = 1U << (LED_PIN + 16U);
}

static void diagnostic_blinks(uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; index++)
    {
        GPIOA->BSRR = 1U << LED_PIN;
        HAL_Delay(120U);
        GPIOA->BSRR = 1U << (LED_PIN + 16U);
        HAL_Delay(180U);
    }
}

static void fault_delay_ms(uint32_t milliseconds)
{
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = 5U;

    while (milliseconds-- > 0U)
    {
        while ((SysTick->CTRL & (1U << 16)) == 0U)
        {
        }
    }

    SysTick->CTRL = 0U;
    SysTick->VAL = 0U;
}

static void fault_blink_forever(uint32_t count)
{
    uint32_t index;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER =
        (GPIOA->MODER & ~GPIO_MODER_MODER15_Msk) |
        GPIO_MODER_MODER15_0;

    for (;;)
    {
        for (index = 0U; index < count; index++)
        {
            GPIOA->BSRR = 1U << LED_PIN;
            fault_delay_ms(350U);
            GPIOA->BSRR = 1U << (LED_PIN + 16U);
            fault_delay_ms(350U);
        }

        fault_delay_ms(2000U);
    }
}

static void fault_blink_hex(uint32_t address)
{
    static const uint8_t shifts[4] = {12U, 8U, 4U, 0U};
    uint32_t group;
    uint32_t index;
    uint8_t digit;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER =
        (GPIOA->MODER & ~GPIO_MODER_MODER15_Msk) |
        GPIO_MODER_MODER15_0;

    for (;;)
    {
        fault_delay_ms(5000U);

        for (group = 0U; group < 4U; group++)
        {
            digit = (uint8_t)((address >> shifts[group]) & 0x0FU);
            for (index = 0U; index < digit; index++)
            {
                GPIOA->BSRR = 1U << LED_PIN;
                fault_delay_ms(250U);
                GPIOA->BSRR = 1U << (LED_PIN + 16U);
                fault_delay_ms(250U);
            }

            fault_delay_ms(3000U);
        }

        fault_delay_ms(5000U);
    }
}

void fault_handler_c(uint32_t *stack_frame)
{
    __disable_irq();
    fault_blink_hex(stack_frame[6]);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main(void)
{
    uint32_t last_toggle;
    uint8_t ready_sent = 0U;
    uint8_t receive_buffer[128];

    HAL_Init();
    SCB->SHCSR |=
        SCB_SHCSR_MEMFAULTENA_Msk |
        SCB_SHCSR_BUSFAULTENA_Msk |
        SCB_SHCSR_USGFAULTENA_Msk;

    if (system_clock_config() == 0U)
    {
        fault_blink_forever(5U);
    }

    led_init();
    diagnostic_blinks(2U);

    if (usb_cdc_init() != 0U)
    {
        fault_blink_forever(15U);
    }
    diagnostic_blinks(3U);

    last_toggle = HAL_GetTick();
    GPIOA->BSRR = 1U << LED_PIN;

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t received;
        uint32_t index;

        if (usb_cdc_is_connected())
        {
            if ((ready_sent == 0U) &&
                (usb_cdc_write_string("JHEF722PRO ready\r\n") != 0U))
            {
                ready_sent = 1U;
            }

            received = usb_cdc_read(receive_buffer, sizeof(receive_buffer));
            if (received != 0U)
            {
                (void)usb_cdc_write(receive_buffer, received);
            }
        }
        else if (usb_cdc_init_failure != 0U)
        {
            uint32_t diagnostic_code = 20U + usb_cdc_init_failure;
            for (index = 0U; index < diagnostic_code; index++)
            {
                GPIOA->BSRR = 1U << LED_PIN;
                fault_delay_ms(220U);
                GPIOA->BSRR = 1U << (LED_PIN + 16U);
                fault_delay_ms(220U);
            }
            fault_delay_ms(2000U);
            continue;
        }

        if ((now - last_toggle) >= 500U)
        {
            last_toggle = now;
            if ((GPIOA->ODR & (1U << LED_PIN)) != 0U)
            {
                GPIOA->BSRR = 1U << (LED_PIN + 16U);
            }
            else
            {
                GPIOA->BSRR = 1U << LED_PIN;
            }
        }
    }
}
