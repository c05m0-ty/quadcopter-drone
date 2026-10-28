#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "stm32f7xx_hal.h"
#include <string.h>

#define USBD_MAX_NUM_INTERFACES       2
#define USBD_MAX_NUM_CONFIGURATION    1
#define USBD_MAX_STR_DESC_SIZ         0x100
#define USBD_SUPPORT_USER_STRING      0
#define USBD_SELF_POWERED             0
#define USBD_DEBUG_LEVEL              0

/** @brief 从 USB 静态内存池返回一个分配块。 */
void *USBD_malloc_impl(uint32_t size);

/** @brief USB 静态内存池的释放接口，复位后允许复用该块。 */
void USBD_free_impl(void *pointer);

#define USBD_malloc                   USBD_malloc_impl
#define USBD_free                     USBD_free_impl
#define USBD_memset                   memset
#define USBD_memcpy                   memcpy

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)              do { } while (0)
#else
#define USBD_UsrLog(...)              do { } while (0)
#endif

#define USBD_ErrLog(...)             do { } while (0)
#define USBD_DbgLog(...)             do { } while (0)

#endif
