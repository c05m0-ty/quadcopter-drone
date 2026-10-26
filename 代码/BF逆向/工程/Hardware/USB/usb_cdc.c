#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usb_cdc.h"

#define USB_CDC_RX_RING_SIZE       2048U
#define USB_CDC_TX_TIMEOUT_MS      100U

extern volatile uint32_t usb_fault_stage;
extern volatile uint8_t usb_cdc_class_init_seen;

USBD_HandleTypeDef USBD_Device;

volatile uint8_t usb_cdc_class_init_seen;
volatile uint32_t usb_cdc_init_failure;
static USBD_CDC_HandleTypeDef cdc_handle;

static uint8_t rx_ring[USB_CDC_RX_RING_SIZE];
static uint8_t cdc_rx_buffer[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t cdc_tx_buffer[CDC_DATA_FS_MAX_PACKET_SIZE];
static volatile uint32_t rx_head;
static volatile uint32_t rx_tail;
static volatile uint8_t host_configured;

static USBD_CDC_LineCodingTypeDef line_coding =
{
    115200U,
    0U,
    0U,
    8U
};

static int8_t USB_CDC_Init(void);
static int8_t USB_CDC_DeInit(void);
static int8_t USB_CDC_Control(uint8_t command, uint8_t *buffer, uint16_t length);
static int8_t USB_CDC_Receive(uint8_t *buffer, uint32_t *length);

static USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
    USB_CDC_Init,
    USB_CDC_DeInit,
    USB_CDC_Control,
    USB_CDC_Receive
};

uint8_t usb_cdc_class_init_direct(USBD_HandleTypeDef *device, uint8_t configuration);

static uint8_t cdc_deinit_proxy(USBD_HandleTypeDef *device, uint8_t configuration)
{
    return USBD_CDC.DeInit(device, configuration);
}

static uint8_t cdc_setup_proxy(USBD_HandleTypeDef *device, USBD_SetupReqTypedef *request)
{
    return USBD_CDC.Setup(device, request);
}

static uint8_t cdc_rx_ready_proxy(USBD_HandleTypeDef *device)
{
    return USBD_CDC.EP0_RxReady(device);
}

static uint8_t cdc_data_in_proxy(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return USBD_CDC.DataIn(device, endpoint);
}

static uint8_t cdc_data_out_proxy(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return USBD_CDC.DataOut(device, endpoint);
}

static uint8_t *cdc_hs_config_proxy(uint16_t *length)
{
    return USBD_CDC.GetHSConfigDescriptor(length);
}

static uint8_t *cdc_fs_config_proxy(uint16_t *length)
{
    return USBD_CDC.GetFSConfigDescriptor(length);
}

static uint8_t *cdc_other_speed_proxy(uint16_t *length)
{
    return USBD_CDC.GetOtherSpeedConfigDescriptor(length);
}

static uint8_t *cdc_qualifier_proxy(uint16_t *length)
{
    return USBD_CDC.GetDeviceQualifierDescriptor(length);
}

static const USBD_ClassTypeDef traced_cdc_class =
{
    usb_cdc_class_init_direct,
    cdc_deinit_proxy,
    cdc_setup_proxy,
    0,
    cdc_rx_ready_proxy,
    cdc_data_in_proxy,
    cdc_data_out_proxy,
    0,
    0,
    0,
    cdc_hs_config_proxy,
    cdc_fs_config_proxy,
    cdc_other_speed_proxy,
    cdc_qualifier_proxy
};

uint8_t usb_cdc_class_init_direct(USBD_HandleTypeDef *device, uint8_t configuration)
{
    usb_fault_stage = 110U;
    usb_cdc_init_failure = 0U;
    usb_cdc_class_init_seen = 1U;
    (void)configuration;

    if (USBD_LL_OpenEP(device, CDC_IN_EP, USBD_EP_TYPE_BULK,
                       CDC_DATA_FS_IN_PACKET_SIZE) != USBD_OK)
    {
        usb_cdc_init_failure = 1U;
        return 1U;
    }
    usb_fault_stage = 111U;
    if (USBD_LL_OpenEP(device, CDC_OUT_EP, USBD_EP_TYPE_BULK,
                       CDC_DATA_FS_OUT_PACKET_SIZE) != USBD_OK)
    {
        usb_cdc_init_failure = 2U;
        return 1U;
    }
    usb_fault_stage = 112U;
    if (USBD_LL_OpenEP(device, CDC_CMD_EP, USBD_EP_TYPE_INTR,
                       CDC_CMD_PACKET_SIZE) != USBD_OK)
    {
        usb_cdc_init_failure = 3U;
        return 1U;
    }
    usb_fault_stage = 113U;

    device->pClassData = &cdc_handle;
    USBD_memset(&cdc_handle, 0, sizeof(cdc_handle));
    cdc_handle.TxState = 0U;
    cdc_handle.RxState = 0U;

    usb_fault_stage = 119U;
    if (USBD_CDC_SetTxBuffer(device, cdc_tx_buffer, 0U) != USBD_OK)
    {
        usb_cdc_init_failure = 4U;
        return 1U;
    }
    usb_fault_stage = 120U;
    if (USBD_CDC_SetRxBuffer(device, cdc_rx_buffer) != USBD_OK)
    {
        usb_cdc_init_failure = 5U;
        return 1U;
    }

    host_configured = 1U;
    usb_fault_stage = 121U;
    if (USBD_LL_PrepareReceive(device, CDC_OUT_EP, cdc_rx_buffer,
                               CDC_DATA_FS_OUT_PACKET_SIZE) != USBD_OK)
    {
        usb_cdc_init_failure = 6U;
        return 1U;
    }

    usb_fault_stage = 122U;
    return 0U;
}

static uint8_t cdc_transmit_packet(const uint8_t *data, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc;
    uint32_t timeout = USB_CDC_TX_TIMEOUT_MS;

    if (!usb_cdc_is_connected())
    {
        return 1U;
    }

    cdc = (USBD_CDC_HandleTypeDef *)USBD_Device.pClassData;
    if (cdc == 0)
    {
        return 1U;
    }

    while ((cdc->TxState != 0U) && (timeout-- > 0U))
    {
        HAL_Delay(1U);
    }
    if (cdc->TxState != 0U)
    {
        return 1U;
    }

    if (USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)data, length) != USBD_OK)
    {
        return 1U;
    }

    return (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK) ? 0U : 1U;
}

static int8_t USB_CDC_Init(void)
{
    usb_fault_stage = 44U;
    USBD_CDC_SetTxBuffer(&USBD_Device, cdc_tx_buffer, 0U);
    usb_fault_stage = 45U;
    USBD_CDC_SetRxBuffer(&USBD_Device, cdc_rx_buffer);
    host_configured = 1U;
    return 0;
}

static int8_t USB_CDC_DeInit(void)
{
    host_configured = 0U;
    return 0;
}

static int8_t USB_CDC_Control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    switch (command)
    {
    case CDC_SET_LINE_CODING:
        if (length >= 7U)
        {
            line_coding.bitrate =
                (uint32_t)buffer[0] |
                ((uint32_t)buffer[1] << 8) |
                ((uint32_t)buffer[2] << 16) |
                ((uint32_t)buffer[3] << 24);
            line_coding.format = buffer[4];
            line_coding.paritytype = buffer[5];
            line_coding.datatype = buffer[6];
        }
        break;

    case CDC_GET_LINE_CODING:
        buffer[0] = (uint8_t)line_coding.bitrate;
        buffer[1] = (uint8_t)(line_coding.bitrate >> 8);
        buffer[2] = (uint8_t)(line_coding.bitrate >> 16);
        buffer[3] = (uint8_t)(line_coding.bitrate >> 24);
        buffer[4] = line_coding.format;
        buffer[5] = line_coding.paritytype;
        buffer[6] = line_coding.datatype;
        break;

    case CDC_SET_CONTROL_LINE_STATE:
        host_configured = 1U;
        break;

    default:
        break;
    }

    return 0;
}

static int8_t USB_CDC_Receive(uint8_t *buffer, uint32_t *length)
{
    uint32_t index;

    for (index = 0U; index < *length; index++)
    {
        uint32_t next = (rx_head + 1U) & (USB_CDC_RX_RING_SIZE - 1U);
        if (next == rx_tail)
        {
            break;
        }
        rx_ring[rx_head] = buffer[index];
        rx_head = next;
    }

    USBD_CDC_ReceivePacket(&USBD_Device);
    return 0;
}

uint8_t usb_cdc_init(void)
{
    USBD_ClassTypeDef *class_pointer = (USBD_ClassTypeDef *)&traced_cdc_class;

    rx_head = 0U;
    rx_tail = 0U;
    host_configured = 0U;
    usb_cdc_class_init_seen = 0U;

    if (USBD_Init(&USBD_Device, &USBD_Desc, 0U) != USBD_OK)
    {
        return 1U;
    }
    if (USBD_RegisterClass(&USBD_Device, class_pointer) != USBD_OK)
    {
        return 2U;
    }
    if ((USBD_Device.pClass != class_pointer) ||
        (USBD_Device.pClass->Init != usb_cdc_class_init_direct))
    {
        return 3U;
    }
    if (USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops) != USBD_OK)
    {
        return 4U;
    }
    if (USBD_Device.pUserData != &USBD_CDC_fops)
    {
        return 5U;
    }

    USBD_Start(&USBD_Device);
    return (USBD_Device.dev_state == USBD_STATE_DEFAULT) ? 0U : 6U;
}

uint8_t usb_cdc_is_connected(void)
{
    return (host_configured != 0U) &&
           (USBD_Device.dev_state == USBD_STATE_CONFIGURED);
}

uint32_t usb_cdc_read(uint8_t *data, uint32_t length)
{
    uint32_t count = 0U;

    if ((data == 0) || (length == 0U))
    {
        return 0U;
    }

    while ((count < length) && (rx_tail != rx_head))
    {
        data[count++] = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1U) & (USB_CDC_RX_RING_SIZE - 1U);
    }

    return count;
}

uint32_t usb_cdc_write(const uint8_t *data, uint32_t length)
{
    uint32_t written = 0U;

    if ((data == 0) || !usb_cdc_is_connected())
    {
        return 0U;
    }

    while (written < length)
    {
        uint32_t remaining = length - written;
        uint16_t chunk = (remaining > CDC_DATA_FS_MAX_PACKET_SIZE)
            ? CDC_DATA_FS_MAX_PACKET_SIZE
            : (uint16_t)remaining;

        if (cdc_transmit_packet(&data[written], chunk) != 0U)
        {
            break;
        }

        written += chunk;
    }

    return written;
}

uint32_t usb_cdc_write_string(const char *text)
{
    uint32_t length = 0U;

    if (text == 0)
    {
        return 0U;
    }

    while (text[length] != '\0')
    {
        length++;
    }

    return usb_cdc_write((const uint8_t *)text, length);
}

