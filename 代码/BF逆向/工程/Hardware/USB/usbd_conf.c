#include "usbd_core.h"
#include "usbd_conf.h"

PCD_HandleTypeDef hpcd;
volatile uint32_t usb_fault_stage;
volatile uint32_t usb_last_setup_code;

static uint64_t usbd_memory_pool[128];

void *USBD_malloc_impl(uint32_t size)
{
    if (size > sizeof(usbd_memory_pool))
    {
        return 0;
    }

    /* This project has one CDC class allocation. Reuse this block after resets. */
    return usbd_memory_pool;
}

void USBD_free_impl(void *pointer)
{
    (void)pointer;
}

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

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB_OTG_FS)
    {
        return;
    }

    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
}

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

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    usb_fault_stage = 3U;
    USBD_LL_DataOutStage(pcd->pData, epnum, pcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    usb_fault_stage = 4U;
    USBD_LL_DataInStage(pcd->pData, epnum, pcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 5U;
    USBD_LL_SOF(pcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 92U;
    USBD_LL_Reset(pcd->pData);
    USBD_LL_SetSpeed(pcd->pData, USBD_SPEED_FULL);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 98U;
    USBD_LL_Suspend(pcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 99U;
    USBD_LL_Resume(pcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(pcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(pcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 97U;
    USBD_LL_DevConnected(pcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *pcd)
{
    usb_fault_stage = 96U;
    USBD_LL_DevDisconnected(pcd->pData);
}

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

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *device)
{
    return (HAL_PCD_DeInit(device->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *device)
{
    return (HAL_PCD_Start(device->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *device)
{
    return (HAL_PCD_Stop(device->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

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

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_Close(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_Flush(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_SetStall(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return (HAL_PCD_EP_ClrStall(device->pData, endpoint) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    PCD_HandleTypeDef *pcd = device->pData;

    if ((endpoint & 0x80U) != 0U)
    {
        return pcd->IN_ep[endpoint & 0x7FU].is_stall;
    }

    return pcd->OUT_ep[endpoint & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *device, uint8_t address)
{
    return (HAL_PCD_SetAddress(device->pData, address) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *device,
                                    uint8_t endpoint,
                                    uint8_t *buffer,
                                    uint16_t size)
{
    return (HAL_PCD_EP_Transmit(device->pData, endpoint, buffer, size) == HAL_OK)
        ? USBD_OK : USBD_FAIL;
}

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

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *device, uint8_t endpoint)
{
    return HAL_PCD_EP_GetRxCount(device->pData, endpoint);
}

void USBD_LL_Delay(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

void OTG_FS_IRQHandler(void)
{
    usb_fault_stage = 10U;
    HAL_PCD_IRQHandler(&hpcd);
    usb_fault_stage = 0U;
}
