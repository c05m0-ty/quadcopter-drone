#ifndef STM32F7XX_HAL_CONF_H
#define STM32F7XX_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

#if !defined(HSE_VALUE)
#define HSE_VALUE                   8000000U
#endif
#define HSE_STARTUP_TIMEOUT         0x5000U
#define HSI_VALUE                   16000000U
#define LSI_VALUE                   32000U
#define LSE_VALUE                   32768U
#define LSE_STARTUP_TIMEOUT         0x5000U
#define EXTERNAL_CLOCK_VALUE        12288000U

#define VDD_VALUE                   3300U
#define TICK_INT_PRIORITY           0x0FU
#define USE_RTOS                    0U
#define PREFETCH_ENABLE             1U
#define INSTRUCTION_CACHE_ENABLE    1U
#define DATA_CACHE_ENABLE           1U
#define USE_SPI_CRC                 0U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f7xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f7xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f7xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f7xx_hal_cortex.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f7xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f7xx_hal_pwr.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f7xx_hal_pcd.h"
#endif

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#endif
