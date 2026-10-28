#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usb_cdc.h"

#define USB_CDC_RX_RING_SIZE       2048U
#define USB_CDC_TX_CHUNK_SIZE      63U

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

/**
 * @brief CDC 类反初始化代理，将 HAL 回调转给标准 CDC 类实现。
 * @param device USB 设备句柄。
 * @param configuration USB 配置编号。
 * @return USBD_OK 或底层返回的状态。
 */
static uint8_t cdc_deinit_proxy(USBD_HandleTypeDef *device, uint8_t configuration)
{
    return USBD_CDC.DeInit(device, configuration);
}

/**
 * @brief CDC 控制请求代理，将 USB Setup 包转给标准 CDC 类处理。
 * @param device USB 设备句柄。
 * @param request USB 标准或类控制请求。
 * @return USBD_OK 或底层返回的状态。
 */
static uint8_t cdc_setup_proxy(USBD_HandleTypeDef *device, USBD_SetupReqTypedef *request)
{
    return USBD_CDC.Setup(device, request);
}

/**
 * @brief CDC 控制端点接收完成代理。
 * @param device USB 设备句柄。
 * @return USBD_OK 或底层返回的状态。
 */
static uint8_t cdc_rx_ready_proxy(USBD_HandleTypeDef *device)
{
    return USBD_CDC.EP0_RxReady(device);
}

/**
 * @brief CDC 数据 IN 端点完成代理。
 * @param device USB 设备句柄。
 * @param endpoint 完成的 IN 端点编号。
 * @return USBD_OK 或底层返回的状态。
 */
static uint8_t cdc_data_in_proxy(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return USBD_CDC.DataIn(device, endpoint);
}

/**
 * @brief CDC 数据 OUT 端点完成代理。
 * @param device USB 设备句柄。
 * @param endpoint 完成的 OUT 端点编号。
 * @return USBD_OK 或底层返回的状态。
 */
static uint8_t cdc_data_out_proxy(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return USBD_CDC.DataOut(device, endpoint);
}

/**
 * @brief 获取 CDC 高速配置描述符。
 * @param length 输出描述符长度。
 * @return 描述符缓冲区指针。
 */
static uint8_t *cdc_hs_config_proxy(uint16_t *length)
{
    return USBD_CDC.GetHSConfigDescriptor(length);
}

/**
 * @brief 获取 CDC 全速配置描述符。
 * @param length 输出描述符长度。
 * @return 描述符缓冲区指针。
 */
static uint8_t *cdc_fs_config_proxy(uint16_t *length)
{
    return USBD_CDC.GetFSConfigDescriptor(length);
}

/**
 * @brief 获取 CDC 其他速度配置描述符。
 * @param length 输出描述符长度。
 * @return 描述符缓冲区指针。
 */
static uint8_t *cdc_other_speed_proxy(uint16_t *length)
{
    return USBD_CDC.GetOtherSpeedConfigDescriptor(length);
}

/**
 * @brief 获取 USB 设备限定描述符。
 * @param length 输出描述符长度。
 * @return 描述符缓冲区指针。
 */
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

/**
 * @brief 手动初始化 CDC 数据和控制端点，为故障诊断记录初始化阶段。
 * @param device USB 设备句柄。
 * @param configuration USB 配置编号，本实现不使用。
 * @return 0 表示端点初始化成功，1 表示某一阶段失败。
 */
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

/**
 * @brief 在发送端点空闲时提交一个不超过 63 字节的 CDC 数据包。
 * @param data 待发送数据。
 * @param length 数据长度。
 * @return 0 表示数据包已交给 USB 栈，1 表示未连接、端点忙或提交失败。
 * @note 本函数不等待端点空闲；忙时由上层丢弃该帧，避免阻塞传感器采集。
 */
static uint8_t cdc_transmit_packet(const uint8_t *data, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc;

    if (!usb_cdc_is_connected())
    {
        return 1U;
    }

    cdc = (USBD_CDC_HandleTypeDef *)USBD_Device.pClassData;
    if (cdc == 0)
    {
        return 1U;
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

/**
 * @brief CDC 类初始化回调，为发送和接收准备端点缓冲区。
 * @return 固定返回 0，表示初始化成功。
 */
static int8_t USB_CDC_Init(void)
{
    usb_fault_stage = 44U;
    USBD_CDC_SetTxBuffer(&USBD_Device, cdc_tx_buffer, 0U);
    usb_fault_stage = 45U;
    USBD_CDC_SetRxBuffer(&USBD_Device, cdc_rx_buffer);
    host_configured = 1U;
    return 0;
}

/**
 * @brief CDC 类反初始化回调，清除主机已配置状态。
 * @return 固定返回 0。
 */
static int8_t USB_CDC_DeInit(void)
{
    host_configured = 0U;
    return 0;
}

/**
 * @brief 处理 CDC 串口线编码和控制线状态类请求。
 * @param command CDC 类请求代码。
 * @param buffer 请求数据缓冲区。
 * @param length 请求数据长度。
 * @return 固定返回 0。
 * @note 虚拟串口会记录波特率等参数，但 USB 全速链路速率不受该值影响。
 */
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

/**
 * @brief 将主机发来的 OUT 数据包写入软件环形缓冲区，并重新准备接收。
 * @param buffer USB 端点接收到的数据。
 * @param length 输入输出参数：进入时为数据长度，退出时保持不变。
 * @return 固定返回 0，表示接收回调已处理。
 */
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

/**
 * @brief 初始化 USB 设备栈、注册 CDC 类并启动 USB 外设。
 * @retval 0 USB 栈已启动。
 * @retval 1 USBD_Init 失败。
 * @retval 2 CDC 类注册失败。
 * @retval 3 CDC 类回调注册结果异常。
 * @retval 4 CDC 接口注册失败。
 * @retval 5 CDC 私有数据注册结果异常。
 * @retval 6 USB 启动后的初始状态异常。
 */
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

/**
 * @brief 判断 USB 主机是否已完成配置，可以进行虚拟串口收发。
 * @return 1 表示已连接并配置，0 表示尚未就绪。
 */
uint8_t usb_cdc_is_connected(void)
{
    return (host_configured != 0U) &&
           (USBD_Device.dev_state == USBD_STATE_CONFIGURED);
}

/**
 * @brief 从 CDC 接收环形缓冲区读取数据，不阻塞。
 * @param data 输出缓冲区。
 * @param length 最多读取字节数。
 * @return 实际读取的字节数。
 */
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

/**
 * @brief 将数据按 63 字节分块发送到 USB CDC 虚拟串口。
 * @param data 待发送数据。
 * @param length 数据长度。
 * @return 实际提交的字节数；连接中断或发送端点忙时会丢弃当前帧。
 */
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
        uint16_t chunk = (remaining > USB_CDC_TX_CHUNK_SIZE)
            ? USB_CDC_TX_CHUNK_SIZE
            : (uint16_t)remaining;

        if (cdc_transmit_packet(&data[written], chunk) != 0U)
        {
            break;
        }

        written += chunk;
    }

    return written;
}

/**
 * @brief 计算字符串长度并通过 CDC 虚拟串口发送。
 * @param text 以 '\0' 结尾的待发送字符串。
 * @return 实际完整发送的字节数。
 */
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

