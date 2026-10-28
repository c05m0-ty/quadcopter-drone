#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

/** @brief 初始化 USB 设备栈和 CDC 虚拟串口。 */
uint8_t usb_cdc_init(void);

/** @brief 查询 USB 主机是否已连接并完成配置。 */
uint8_t usb_cdc_is_connected(void);

/** @brief 从虚拟串口接收环形缓冲区读取数据。 */
uint32_t usb_cdc_read(uint8_t *data, uint32_t length);

/** @brief 向虚拟串口发送指定长度的数据。 */
uint32_t usb_cdc_write(const uint8_t *data, uint32_t length);

/** @brief 向虚拟串口发送以 '\0' 结尾的字符串。 */
uint32_t usb_cdc_write_string(const char *text);

#endif
