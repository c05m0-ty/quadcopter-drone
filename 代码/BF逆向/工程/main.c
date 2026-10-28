#include "stm32f7xx_hal.h"
#include "gyro_attitude.h"
#include "gyro_sensor.h"
#include "usb_cdc.h"

#define LED_PIN 15U
#define GYRO_STALE_TIMEOUT_MS     100U
#define GYRO_RECOVERY_RETRY_MS    500U

extern volatile uint32_t usb_cdc_init_failure;

/**
 * @brief 配置板级系统时钟、外设总线和 USB 所需的 48 MHz 时钟。
 * @return 1 表示配置成功；0 表示任一阶段配置失败。
 */
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

/**
 * @brief 将 PA15 配置为推挽输出，并先关闭板载 LED。
 */
static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    GPIOA->MODER =
        (GPIOA->MODER & ~GPIO_MODER_MODER15_Msk) |
        GPIO_MODER_MODER15_0;

    GPIOA->BSRR = 1U << (LED_PIN + 16U);
}

/**
 * @brief 阻塞式闪烁 LED，用于开机阶段显示诊断进度。
 * @param count 闪烁次数。
 */
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

/**
 * @brief 将以 '\0' 结尾的字符串追加到输出缓冲区。
 * @param buffer 目标缓冲区。
 * @param position 当前写入位置。
 * @param text 要追加的字符串。
 * @return 追加完成后的新写入位置；调用者需保证缓冲区足够大。
 */
static uint32_t append_text(char *buffer, uint32_t position, const char *text)
{
    while (*text != '\0')
    {
        buffer[position++] = *text++;
    }
    return position;
}

/**
 * @brief 将有符号十进制整数追加到输出缓冲区。
 * @param buffer 目标缓冲区。
 * @param position 当前写入位置。
 * @param value 要格式化的整数。
 * @return 追加完成后的新写入位置。
 */
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

/**
 * @brief 将一个字节按两位大写十六进制数追加到输出缓冲区。
 * @param buffer 目标缓冲区。
 * @param position 当前写入位置。
 * @param value 要格式化的字节。
 * @return 追加完成后的新写入位置。
 */
static uint32_t append_hex_byte(char *buffer,
                                uint32_t position,
                                uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";

    buffer[position++] = digits[value >> 4];
    buffer[position++] = digits[value & 0x0FU];
    return position;
}

/**
 * @brief 将浮点数以四位小数格式追加到输出缓冲区。
 * @param buffer 目标缓冲区。
 * @param position 当前写入位置。
 * @param value 要格式化的浮点数。
 * @return 追加完成后的新写入位置。
 * @note 此函数用于生成 VOFA+ FireWater 所需的文本数据。
 */
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

/**
 * @brief 将四元数格式化为 VOFA+ FireWater 的 w,x,y,z 单行文本。
 * @param buffer 输出缓冲区，末尾会写入 '\0'。
 * @param attitude 待格式化的四元数姿态。
 */
static void format_attitude_line(char *buffer,
                                 const gyro_attitude_t *attitude)
{
    uint32_t length;

    length = append_fixed4(buffer, 0U, attitude->w);
    length = append_text(buffer, length, ",");
    length = append_fixed4(buffer, length, attitude->x);
    length = append_text(buffer, length, ",");
    length = append_fixed4(buffer, length, attitude->y);
    length = append_text(buffer, length, ",");
    length = append_fixed4(buffer, length, attitude->z);
    length = append_text(buffer, length, "\r\n");
    buffer[length] = '\0';
}

/**
 * @brief 在故障处理路径中使用裸 SysTick 产生毫秒级阻塞延时。
 * @param milliseconds 需要延时的毫秒数。
 * @note 该函数不依赖 HAL 时基，可在主循环或普通故障路径中调用。
 */
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

/**
 * @brief 进入不可恢复的循环故障状态，按指定次数反复闪烁 LED。
 * @param count 每轮闪烁次数。
 */
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

/**
 * @brief 以四组十六进制数字反复闪烁故障地址，便于无调试器时定位异常。
 * @param address 要显示的低 16 位值，按从高到低共四组半字节闪烁。
 */
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

/**
 * @brief Cortex-M 硬件异常入口，停用中断并通过 LED 显示出错地址。
 * @param stack_frame 硬件异常时保存的寄存器栈帧。
 */
void fault_handler_c(uint32_t *stack_frame)
{
    __disable_irq();
    fault_blink_hex(stack_frame[6]);
}

/**
 * @brief SysTick 中断服务函数，为 HAL 提供 1 ms 系统节拍。
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief 程序主入口：初始化时钟、LED、USB 和传感器，并运行姿态输出主循环。
 * @return 正常运行时不会返回。
 */
int main(void)
{
    /* 主循环使用的定时与通信状态。 */
    uint32_t last_toggle;
    uint32_t last_output_tick = 0U;
    uint32_t dma_start_tick = 0U;
    uint32_t status_wait_tick = 0U;
    uint32_t last_sample_tick = 0U;
    uint32_t last_recovery_tick = 0U;
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

    /* 初始化 HAL，并开启内存、总线和用法异常，便于故障时定位。 */
    HAL_Init();
    SCB->SHCSR |=
        SCB_SHCSR_MEMFAULTENA_Msk |
        SCB_SHCSR_BUSFAULTENA_Msk |
        SCB_SHCSR_USGFAULTENA_Msk;

    /* 时钟是 USB 和 SPI 正常工作的前提，失败时用 5 次快闪报错。 */
    if (system_clock_config() == 0U)
    {
        fault_blink_forever(5U);
    }

    /* 初始化调试 LED；2 次闪烁表示系统时钟已配置完成。 */
    led_init();
    diagnostic_blinks(2U);

    /* 尽早初始化 USB，失败时用 15 次快闪报错；成功则再闪 3 次。 */
    if (usb_cdc_init() != 0U)
    {
        fault_blink_forever(15U);
    }
    diagnostic_blinks(3U);

    gyro_status = gyro_sensor_init(&gyro_identity);

    /* 传感器初始化完成，LED 常亮表示程序已经进入主循环。 */
    last_toggle = HAL_GetTick();
    GPIOA->BSRR = 1U << LED_PIN;

    for (;;)
    {
        /* 每轮读取一次毫秒时基，后面的超时和周期判断都基于它。 */
        uint32_t now = HAL_GetTick();
        uint32_t received;
        uint32_t index;

        if (usb_cdc_is_connected())
        {
            /* 主机首次连接时只记录一次起始时刻，供 5 秒等待使用。 */
            if (status_wait_started == 0U)
            {
                status_wait_tick = now;
                status_wait_started = 1U;
            }

            /*
             * 等待 5 秒再启用 FIFO，给串口工具留出打开和连接时间，
             * 同时避免主机未就绪时 FIFO 先积满数据。
             */
            if ((ready_sent == 0U) &&
                ((now - status_wait_tick) >= 5000U))
            {
                char status_message[160];
                uint32_t length = 0U;

                /* 传感器初始化正常时启动连续 FIFO；失败则转换为配置错误。 */
                if ((gyro_status == GYRO_SENSOR_OK) &&
                    (gyro_sensor_enable_fifo() != GYRO_SENSOR_OK))
                {
                    gyro_status = GYRO_SENSOR_CONFIG_FAIL;
                }

                if (gyro_status != GYRO_SENSOR_OK)
                {
                    /* 组织一条明文错误信息，包错误码及错误 ID 便于排查。 */
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
                    /* 姿态从零开始，等收到首批有效陀螺仪数据后再对外输出。 */
                    gyro_attitude_reset();
                    last_output_tick = now;
                    last_sample_tick = now;
                    attitude_valid = 0U;
                    latest_accel_valid = 0U;
                    ready_sent = 1U;
                }

                if (gyro_status != GYRO_SENSOR_OK)
                {
                    /* 错误信息只发送一次；ready_sent 同时阻止重复启动 FIFO。 */
                    status_message[length] = '\0';
                    (void)usb_cdc_write_string(status_message);
                    ready_sent = 1U;
                }
            }

            /*
             * PC3 中断只报告“可以取数”。没有正在进行的 DMA 时，
             * 才根据 FIFO 水位启动下一批 SPI DMA 读取。
             */
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

            /*
             * DMA 是非阻塞的：主循环随后轮询完成状态。若读取报错，
             * 或 5 ms 内仍未完成，则中止本次传输，防止主循环被卡住。
             */
            if (dma_waiting != 0U)
            {
                gyro_sensor_fifo_batch_t batch;
                uint8_t read_status =
                    gyro_sensor_finish_fifo_dma_read(&batch);

                if (read_status == GYRO_SENSOR_OK)
                {
                    uint16_t sample_index;

                    /* FIFO 批次中最多保留最近一次加速度计样本。 */
                    if (batch.accel_valid != 0U)
                    {
                        latest_accel = batch.accel_sample;
                        latest_accel_valid = 1U;
                    }

                    /*
                     * 按原始顺序逐条处理陀螺仪记录，并及时把最新的加速度
                     * 数据附到样本上，供 Mahony 融合修正姿态漂移。
                     */
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

                        /* 传感器采样率约 1 kHz，因此每条记录积分 1 ms。 */
                        gyro_attitude_update(&sample,
                                             1U,
                                             &current_attitude);
                    }

                    if (batch.gyro_count != 0U)
                    {
                        attitude_valid = 1U;
                        last_sample_tick = now;
                    }
                    dma_waiting = 0U;
                }
                else if ((read_status == GYRO_SENSOR_READ_FAIL) ||
                         ((now - dma_start_tick) >= 5U))
                {
                    /* 出错或超时时释放 DMA 和 SPI 片选，恢复下一轮采集。 */
                    gyro_sensor_abort_dma_read();
                    dma_waiting = 0U;
                }
            }

            /*
             * 连续 100 ms 没有新陀螺仪样本时，停止发送旧姿态并重建
             * FIFO/EXTI 数据通路。恢复失败时每 500 ms 重试一次。
             */
            if ((ready_sent != 0U) &&
                (gyro_status == GYRO_SENSOR_OK) &&
                ((now - last_sample_tick) >= GYRO_STALE_TIMEOUT_MS) &&
                ((now - last_recovery_tick) >= GYRO_RECOVERY_RETRY_MS))
            {
                if (dma_waiting != 0U)
                {
                    gyro_sensor_abort_dma_read();
                    dma_waiting = 0U;
                }

                (void)gyro_sensor_recover_fifo();
                attitude_valid = 0U;
                latest_accel_valid = 0U;
                last_sample_tick = now;
                last_recovery_tick = now;
            }

            /*
             * 姿态以约 20 Hz 输出到 VOFA+，而不是按 1 kHz 原样发送，
             * 以免串口和上位机被过量数据压垮。USB 忙时本帧直接丢弃，
             * 50 ms 后再尝试，主循环不会停在发送端点上等待。
             */
            if ((ready_sent != 0U) &&
                (attitude_valid != 0U) &&
                ((now - last_output_tick) >= 50U))
            {
                char attitude_line[64];

                format_attitude_line(attitude_line, &current_attitude);
                last_output_tick = now;
                (void)usb_cdc_write_string(attitude_line);
            }

            /* 接收上位机命令；当前只识别 z/Z，用于把当前姿态设为零点。 */
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
                    /*
                     * 仅改变显示零点，不重置内部姿态；立即发送一次单位
                     * 四元数，避免采集停滞时 z 命令看起来没有响应。
                     */
                    char zero_line[64];

                    gyro_attitude_zero();
                    current_attitude.w = 1.0f;
                    current_attitude.x = 0.0f;
                    current_attitude.y = 0.0f;
                    current_attitude.z = 0.0f;
                    format_attitude_line(zero_line, &current_attitude);
                    if (usb_cdc_write_string(zero_line) != 0U)
                    {
                        last_output_tick = now;
                    }
                    attitude_valid = 0U;
                }
            }
        }
        else if (usb_cdc_init_failure != 0U)
        {
            /*
             * USB 类初始化失败时，以“20 + 错误码”次闪光显示诊断编号，
             * 然后继续循环，便于观察不同阶段的初始化故障。
             */
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

        /* 主循环心跳：在 USB 等待或正常工作期间让 PA15 LED 每 500 ms 翻转。 */
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
