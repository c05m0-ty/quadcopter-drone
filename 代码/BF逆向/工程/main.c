#include "stm32f7xx_hal.h"
#include "gyro_attitude.h"
#include "gyro_sensor.h"
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

static uint32_t append_text(char *buffer, uint32_t position, const char *text)
{
    while (*text != '\0')
    {
        buffer[position++] = *text++;
    }
    return position;
}

static uint32_t append_int(char *buffer, uint32_t position, int32_t value)
{
    char digits[12];
    uint32_t count = 0U;
    uint32_t magnitude;

    if (value < 0)
    {
        buffer[position++] = '-';
        magnitude = (uint32_t)(-value);
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    do
    {
        digits[count++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    }
    while (magnitude != 0U);

    while (count != 0U)
    {
        buffer[position++] = digits[--count];
    }

    return position;
}

static uint32_t append_hex_byte(char *buffer,
                                uint32_t position,
                                uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";

    buffer[position++] = digits[value >> 4];
    buffer[position++] = digits[value & 0x0FU];
    return position;
}

static uint32_t append_fixed4(char *buffer,
                              uint32_t position,
                              float value)
{
    int32_t scaled;
    uint32_t magnitude;

    scaled = (int32_t)((value >= 0.0f)
        ? (value * 10000.0f + 0.5f)
        : (value * 10000.0f - 0.5f));

    if (scaled < 0)
    {
        buffer[position++] = '-';
        magnitude = (uint32_t)(-scaled);
    }
    else
    {
        magnitude = (uint32_t)scaled;
    }

    position = append_int(buffer,
                          position,
                          (int32_t)(magnitude / 10000U));
    buffer[position++] = '.';
    buffer[position++] = (char)('0' + ((magnitude / 1000U) % 10U));
    buffer[position++] = (char)('0' + ((magnitude / 100U) % 10U));
    buffer[position++] = (char)('0' + ((magnitude / 10U) % 10U));
    buffer[position++] = (char)('0' + (magnitude % 10U));
    return position;
}

static void format_attitude_line(char *buffer,
                                 const gyro_attitude_t *attitude)
{
    uint32_t length;

    length = append_fixed4(buffer, 0U, attitude->roll);
    length = append_text(buffer, length, ",");
    length = append_fixed4(buffer, length, attitude->pitch);
    length = append_text(buffer, length, ",");
    length = append_fixed4(buffer, length, attitude->yaw);
    length = append_text(buffer, length, "\r\n");
    buffer[length] = '\0';
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
    uint32_t last_output_tick = 0U;
    uint32_t dma_start_tick = 0U;
    uint32_t status_wait_tick = 0U;
    uint8_t ready_sent = 0U;
    uint8_t attitude_valid = 0U;
    uint8_t dma_waiting = 0U;
    uint8_t status_wait_started = 0U;
    uint8_t receive_buffer[128];
    uint8_t gyro_identity = 0U;
    uint8_t gyro_status;
    gyro_attitude_t current_attitude = {0};
    gyro_sensor_sample_t latest_accel = {0};
    uint8_t latest_accel_valid = 0U;

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

    gyro_status = gyro_sensor_init(&gyro_identity);

    last_toggle = HAL_GetTick();
    GPIOA->BSRR = 1U << LED_PIN;

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t received;
        uint32_t index;

        if (usb_cdc_is_connected())
        {
            if (status_wait_started == 0U)
            {
                status_wait_tick = now;
                status_wait_started = 1U;
            }

            if ((ready_sent == 0U) &&
                ((now - status_wait_tick) >= 5000U))
            {
                char status_message[160];
                uint32_t length = 0U;

                if ((gyro_status == GYRO_SENSOR_OK) &&
                    (gyro_sensor_enable_fifo() != GYRO_SENSOR_OK))
                {
                    gyro_status = GYRO_SENSOR_CONFIG_FAIL;
                }

                if (gyro_status != GYRO_SENSOR_OK)
                {
                    length = append_text(status_message,
                                         0U,
                                         "JHEF722PRO ready\r\n"
                                         "GYRO init failed code=");
                    length = append_int(status_message,
                                        length,
                                        gyro_status);
                    if (gyro_status == GYRO_SENSOR_WRONG_ID)
                    {
                        length = append_text(status_message,
                                             length,
                                             " id=0x");
                        length = append_hex_byte(status_message,
                                                 length,
                                                 gyro_identity);
                    }
                    length = append_text(status_message, length, "\r\n");
                }
                else
                {
                    gyro_attitude_reset();
                    last_output_tick = now;
                    attitude_valid = 0U;
                    latest_accel_valid = 0U;
                    ready_sent = 1U;
                }

                if (gyro_status != GYRO_SENSOR_OK)
                {
                    status_message[length] = '\0';
                    (void)usb_cdc_write_string(status_message);
                    ready_sent = 1U;
                }
            }

            if ((ready_sent != 0U) &&
                (dma_waiting == 0U) &&
                (gyro_sensor_take_data_ready() != 0U))
            {
                if (gyro_sensor_start_fifo_dma_read() == GYRO_SENSOR_OK)
                {
                    dma_waiting = 1U;
                    dma_start_tick = now;
                }
            }

            if (dma_waiting != 0U)
            {
                gyro_sensor_fifo_batch_t batch;
                uint8_t read_status =
                    gyro_sensor_finish_fifo_dma_read(&batch);

                if (read_status == GYRO_SENSOR_OK)
                {
                    uint16_t sample_index;

                    if (batch.accel_valid != 0U)
                    {
                        latest_accel = batch.accel_sample;
                        latest_accel_valid = 1U;
                    }

                    for (sample_index = 0U;
                         sample_index < batch.gyro_count;
                         sample_index++)
                    {
                        gyro_sensor_sample_t sample =
                            batch.gyro_samples[sample_index];

                        if (latest_accel_valid != 0U)
                        {
                            sample.accel_x = latest_accel.accel_x;
                            sample.accel_y = latest_accel.accel_y;
                            sample.accel_z = latest_accel.accel_z;
                        }

                        gyro_attitude_update(&sample,
                                             1U,
                                             &current_attitude);
                    }

                    if (batch.gyro_count != 0U)
                    {
                        attitude_valid = 1U;
                    }
                    dma_waiting = 0U;
                }
                else if ((read_status == GYRO_SENSOR_READ_FAIL) ||
                         ((now - dma_start_tick) >= 5U))
                {
                    gyro_sensor_abort_dma_read();
                    dma_waiting = 0U;
                }
            }

            if ((ready_sent != 0U) &&
                (attitude_valid != 0U) &&
                ((now - last_output_tick) >= 50U))
            {
                char attitude_line[64];

                format_attitude_line(attitude_line, &current_attitude);
                if (usb_cdc_write_string(attitude_line) != 0U)
                {
                    last_output_tick = now;
                }
            }

            received = usb_cdc_read(receive_buffer, sizeof(receive_buffer));
            if (received != 0U)
            {
                uint32_t receive_index;
                uint8_t zero_requested = 0U;

                for (receive_index = 0U;
                     receive_index < received;
                     receive_index++)
                {
                    if ((receive_buffer[receive_index] == 'z') ||
                        (receive_buffer[receive_index] == 'Z'))
                    {
                        zero_requested = 1U;
                    }
                }

                if (zero_requested != 0U)
                {
                    gyro_attitude_zero();
                }
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
