#include "usbd_core.h"
#include "usbd_conf.h"

PCD_HandleTypeDef hpcd;
volatile uint32_t usb_fault_stage;
volatile uint32_t usb_last_setup_code;

static uint64_t usbd_memory_pool[128];

/**
 * @brief USB 设备栈的静态内存分配接口。
 * @param size 请求分配的字节数。
 * @return 大小未超过内存池时返回固定内存池指针，否则返回空指针。
 * @note 当前工程只有一个 CDC 类分配，因此不维护多个分配块。
 */
void *USBD_malloc_impl(uint32_t size)
{
    if (size > sizeof(usbd_memory_pool))
    {
        return 0;
    }

    /* This project has one CDC class allocation. Reuse this block after resets. */
    return usbd_memory_pool;
}

/**
 * @brief USB 设备栈的静态内存释放接口。
 * @param pointer 待释放指针。
 * @note 内存来自静态池且允许复位后复用，因此此处无需实际释放。
 */
void USBD_free_impl(void *pointer)
{
    (void)pointer;
}

/**
 * @brief 初始化 USB OTG FS 使用的 PA11/PA12、时钟和中断。
 * @param pcd PCD 外设句柄；仅处理 USB_OTG_FS。
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
    GPIO_InitTypeDef gpio = {0};

    if (pcd->Instance != USB_OTG_FS)
    {
        return;
    }

    /* Select the 48 MHz PLLSAI output configured during clock init. */
    RCC->DCKCFGR2 |= RCC_DCKCFGR2_CK48MSEL;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
 * @brief 关闭 USB OTG FS 中断和时钟。
 * @param pcd PCD 外设句柄；仅处理 USB_OTG_FS。
 */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB_OTG_FS)
    {
        return;
    }

    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
}

/**
 * @brief 处理 USB Setup 阶段，记录请求类型并转交 USB 设备栈。
 * @param pcd PCD 外设句柄。
 * @note usb_last_setup_code 仅用于无调试器时的故障定位。
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd)
{
    uint8_t request = pcd->Setup[1];
    uint8_t descriptor = pcd->Setup[3];

    if (request == 6U)
    {
        switch (descriptor)
        {
        case 1U:  usb_last_setup_code = 1U; break; /* Device descriptor */
        case 2U:  usb_last_setup_code = 2U; break; /* Configuration descriptor */
        case 3U:  usb_last_setup_code = 3U; break; /* String descriptor */
        case 6U:  usb_last_setup_code = 4U; break; /* Device qualifier */
        case 7U:  usb_last_setup_code = 5U; break; /* Other speed configuration */
        default:  usb_last_setup_code = 13U; break;
        }
    }
    else
    {
        switch (request)
        {
        case 0U:    usb_last_setup_code = 9U; break; /* GET_STATUS */
        case 5U:    usb_last_setup_code = 6U; break; /* SET_ADDRESS */
        case 8U:    usb_last_setup_code = 8U; break; /* GET_CONFIGURATION */
        case 9U:    usb_last_setup_code = 7U; break; /* SET_CONFIGURATION */
        case 0x20U: usb_last_setup_code = 10U; break; /* SET_LINE_CODING */
        case 0x21U: usb_last_setup_code = 11U; break; /* GET_LINE_CODING */
        case 0x22U: usb_last_setup_code = 12U; break; /* SET_CONTROL_LINE_STATE */
        default:    usb_last_setup_code = 13U; break;
        }
    }

    USBD_LL_SetupStage(pcd->pData, (uint8_t *)pcd->Setup);
}

/**
 * @brief 处理 USB OUT 端点完成事件并转交设备栈。
 * @param pcd PCD 外设句柄。
 * @param epnum 完成的 OUT 端点编号。
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    usb_fault_stage = 3U;
    USBD_LL_DataOutStage(pcd->pData, epnum, pcd->OUT_ep[epnum].xfer_buff);
}

/**
 * @brief 处理 USB IN 端点完成事件并转交设备栈。
 * @param pcd PCD 外设句柄。
 * @param epnum 完成的 IN 端点编号。
 */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    usb_fault_stage = 4U;
    USBD_LL_DataInStage(pcd->pData, epnum, pcd->IN_ep[epnum].xfer_buff);
}

/**
 * @brief 处理 USB 帧起始事件并通知 USB 设备栈。
 * @param pcd PCD 外设句柄。
 */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 5U;
    USBD_LL_SOF(pcd->pData);
}

/**
 * @brief 处理 USB 总线复位，并设置设备为全速模式。
 * @param pcd PCD 外设句柄。
 */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 92U;
    USBD_LL_Reset(pcd->pData);
    USBD_LL_SetSpeed(pcd->pData, USBD_SPEED_FULL);
}

/**
 * @brief 处理 USB 总线挂起事件。
 * @param pcd PCD 外设句柄。
 */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 98U;
    USBD_LL_Suspend(pcd->pData);
}

/**
 * @brief 处理 USB 总线恢复事件。
 * @param pcd PCD 外设句柄。
 */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 99U;
    USBD_LL_Resume(pcd->pData);
}

/**
 * @brief 处理同步 OUT 传输未完成事件。
 * @param pcd PCD 外设句柄。
 * @param epnum 未完成的 OUT 端点编号。
 */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(pcd->pData, epnum);
}

/**
 * @brief 处理同步 IN 传输未完成事件。
 * @param pcd PCD 外设句柄。
 * @param epnum 未完成的 IN 端点编号。
 */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(pcd->pData, epnum);
}

/**
 * @brief 处理 USB 设备连接事件并通知设备栈。
 * @param pcd PCD 外设句柄。
 */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 97U;
    USBD_LL_DevConnected(pcd->pData);
}

/**
 * @brief 处理 USB 设备断开事件并通知设备栈。
 * @param pcd PCD 外设句柄。
 */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 96U;
    USBD_LL_DevDisconnected(pcd->pData);
}

/**
 * @brief 初始化 USB OTG FS 外设、全速端点和 FIFO，并关联 USB 设备栈。
 * @param device USB 设备句柄。
 * @return USBD_OK 表示成功，USBD_FAIL 表示 HAL 初始化失败。
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *device)
{
    HAL_StatusTypeDef status;

    hpcd.Instance = USB_OTG_FS;
    hpcd.Init.dev_endpoints = 4U;
    hpcd.Init.use_dedicated_ep1 = 0U;
    hpcd.Init.ep0_mps = 64U;
    hpcd.Init.dma_enable = 0U;
    hpcd.Init.low_power_enable = 0U;
    hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd.Init.Sof_enable = 0U;
    hpcd.Init.speed = PCD_SPEED_FULL;
    hpcd.Init.vbus_sensing_enable = 0U;
    hpcd.Init.lpm_enable = 0U;

    hpcd.pData = device;
    device->pData = &hpcd;
    device->pUserData = 0;

    status = HAL_PCD_Init(&hpcd);
    if (status != HAL_OK)
    {
        return USBD_FAIL;
    }

    HAL_PCDEx_SetRxFiFo(&hpcd, 0x80U);
    HAL_PCDEx_SetTxFiFo(&hpcd, 0U, 0x20U);
    HAL_PCDEx_SetTxFiFo(&hpcd, 1U, 0x40U);
    HAL_PCDEx_SetTxFiFo(&hpcd, 2U, 0x10U);

    return USBD_OK;
}

/**
 * @brief 反初始化底层 USB PCD 外设。
 * @param device USB 设备句柄。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *device)
{
    return (HAL_PCD_DeInit(device->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 启动底层 USB PCD 外设。
 * @param device USB 设备句柄。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *device)
{
    return (HAL_PCD_Start(device->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 停止底层 USB PCD 外设。
 * @param device USB 设备句柄。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *device)
{
    return (HAL_PCD_Stop(device->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 打开一个 USB 端点，并记录诊断阶段。
 * @param device USB 设备句柄。
 * @param endpoint 端点地址。
 * @param type 端点传输类型。
 * @param max_packet 最大包长度。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *device,
                                  uint8_t endpoint,
                                  uint8_t type,
                                  uint16_t max_packet)
{
    if (endpoint == 0x81U)
    {
        usb_fault_stage = 41U;
    }
    else if (endpoint == 0x01U)
    {
        usb_fault_stage = 42U;
    }
    else if (endpoint == 0x82U)
    {
        usb_fault_stage = 43U;
    }
    else
    {
        usb_fault_stage = 40U;
    }

    return (HAL_PCD_EP_Open(device->pData, endpoint, max_packet, type) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 关闭指定 USB 端点。
 * @param device USB 设备句柄。
 * @param endpoint 端点地址。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_Close(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 清空指定 USB 端点的硬件缓冲区。
 * @param device USB 设备句柄。
 * @param endpoint 端点地址。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_Flush(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 将指定 USB 端点置为 STALL 状态。
 * @param device USB 设备句柄。
 * @param endpoint 端点地址。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_SetStall(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 清除指定 USB 端点的 STALL 状态。
 * @param device USB 设备句柄。
 * @param endpoint 端点地址。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_ClrStall(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 查询指定 USB 端点是否处于 STALL 状态。
 * @param device USB 设备句柄。
 * @param endpoint 端点地址，最高位用于区分 IN/OUT。
 * @return 1 表示已 STALL，0 表示未 STALL。
 */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    PCD_HandleTypeDef *pcd = device->pData;

    if ((endpoint & 0x80U) != 0U)
    {
        return pcd->IN_ep[endpoint & 0x7FU].is_stall;
    }

    return pcd->OUT_ep[endpoint & 0x7FU].is_stall;
}

/**
 * @brief 设置 USB 设备地址。
 * @param device USB 设备句柄。
 * @param address 新的 USB 地址。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *device, uint8_t address)
{
    return (HAL_PCD_SetAddress(device->pData, address) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 从指定 IN 端点发送数据。
 * @param device USB 设备句柄。
 * @param endpoint IN 端点地址。
 * @param buffer 发送缓冲区。
 * @param size 发送字节数。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *device,
                                    uint8_t endpoint,
                                    uint8_t *buffer,
                                    uint16_t size)
{
    return (HAL_PCD_EP_Transmit(device->pData, endpoint, buffer, size) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 在指定 OUT 端点准备接收数据。
 * @param device USB 设备句柄。
 * @param endpoint OUT 端点地址。
 * @param buffer 接收缓冲区。
 * @param size 最大接收字节数。
 * @return USBD_OK 或 USBD_FAIL。
 */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *device,
                                          uint8_t endpoint,
                                          uint8_t *buffer,
                                          uint16_t size)
{
    if (endpoint == 0x01U)
    {
        usb_fault_stage = 46U;
    }

    return (HAL_PCD_EP_Receive(device->pData, endpoint, buffer, size) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

/**
 * @brief 获取指定 OUT 端点最近一次实际收到的字节数。
 * @param device USB 设备句柄。
 * @param endpoint OUT 端点地址。
 * @return 接收字节数。
 */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return HAL_PCD_EP_GetRxCount(device->pData, endpoint);
}

/**
 * @brief USB 设备栈使用的毫秒阻塞延时。
 * @param delay_ms 延时毫秒数。
 */
void USBD_LL_Delay(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/**
 * @brief USB OTG FS 中断入口，将硬件事件交给 HAL PCD 状态机处理。
 */
void OTG_FS_IRQHandler(void)
{
    usb_fault_stage = 10U;
    HAL_PCD_IRQHandler(&hpcd);
    usb_fault_stage = 0U;
}
