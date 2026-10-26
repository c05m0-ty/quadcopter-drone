#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

uint8_t usb_cdc_init(void);
uint8_t usb_cdc_is_connected(void);
uint32_t usb_cdc_read(uint8_t *data, uint32_t length);
uint32_t usb_cdc_write(const uint8_t *data, uint32_t length);
uint32_t usb_cdc_write_string(const char *text);

#endif
