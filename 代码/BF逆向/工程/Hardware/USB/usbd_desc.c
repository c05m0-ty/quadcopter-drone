#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USB_CDC_VID                  0x0483U
#define USB_CDC_PID                  0x5740U
#define USB_CDC_LANGID               0x0409U
#define USB_CDC_MANUFACTURER         "STMicroelectronics"
#define USB_CDC_PRODUCT              "JHEF722PRO USB CDC"
#define USB_CDC_CONFIGURATION        "CDC Configuration"
#define USB_CDC_INTERFACE            "CDC Interface"
#define USB_CDC_SERIAL_DESC_SIZE     26U

static uint8_t *device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *langid_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *manufacturer_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *product_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *configuration_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *interface_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef USBD_Desc =
{
    device_descriptor,
    langid_descriptor,
    manufacturer_descriptor,
    product_descriptor,
    serial_descriptor,
    configuration_descriptor,
    interface_descriptor
};

__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
    0x12,
    USB_DESC_TYPE_DEVICE,
    0x00, 0x02,
    0x02,
    0x02,
    0x00,
    USB_MAX_EP0_SIZE,
    LOBYTE(USB_CDC_VID), HIBYTE(USB_CDC_VID),
    LOBYTE(USB_CDC_PID), HIBYTE(USB_CDC_PID),
    0x00, 0x02,
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USB_CDC_LANGID),
    HIBYTE(USB_CDC_LANGID)
};

__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_CDC_SERIAL_DESC_SIZE] __ALIGN_END =
{
    USB_CDC_SERIAL_DESC_SIZE,
    USB_DESC_TYPE_STRING,
    'J', 0U, 'H', 0U, 'E', 0U, 'F', 0U,
    '7', 0U, '2', 0U, '2', 0U, 'P', 0U,
    'R', 0U, 'O', 0U, '0', 0U, '1', 0U
};

__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/**
 * @brief 返回 USB 设备描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 设备描述符缓冲区指针。
 */
static uint8_t *device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_DeviceDesc);
    return USBD_DeviceDesc;
}

/**
 * @brief 返回 USB 语言 ID 描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 语言 ID 描述符缓冲区指针。
 */
static uint8_t *langid_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

/**
 * @brief 生成并返回 USB 制造商字符串描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 已填充的字符串描述符缓冲区指针。
 */
static uint8_t *manufacturer_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USB_CDC_MANUFACTURER, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/**
 * @brief 生成并返回 USB 产品字符串描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 已填充的字符串描述符缓冲区指针。
 */
static uint8_t *product_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USB_CDC_PRODUCT, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/**
 * @brief 返回固定的 USB 序列号字符串描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 序列号字符串描述符缓冲区指针。
 */
static uint8_t *serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = USB_CDC_SERIAL_DESC_SIZE;
    return USBD_StringSerial;
}

/**
 * @brief 生成并返回 USB 配置字符串描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 已填充的字符串描述符缓冲区指针。
 */
static uint8_t *configuration_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USB_CDC_CONFIGURATION, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/**
 * @brief 生成并返回 USB 接口字符串描述符。
 * @param speed 当前 USB 速度，本描述符不区分速度。
 * @param length 输出描述符长度。
 * @return 已填充的字符串描述符缓冲区指针。
 */
static uint8_t *interface_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USB_CDC_INTERFACE, USBD_StrDesc, length);
    return USBD_StrDesc;
}
