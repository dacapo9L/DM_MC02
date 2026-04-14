// 来源于中科大的usb双缓冲

#include "bsp_usb.h"
#include "bsp_dwt.h"

struct Struct_USB_Manage_Object USB0_Manage_Object = {NULL};

/**
 * @brief 初始化USB
 * @param Callback_Function 处理回调函数
 */
void USB_Init(USB_Callback Callback_Function) {
  USB0_Manage_Object.Callback_Function = Callback_Function;

  USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
}

/**
 * @brief 发送数据帧
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length) {
  return (CDC_Transmit_HS(Data, Length));
}

/**
 * @brief 自己写的USB通信下一轮接收开启前回调函数, 非HAL库回调函数
 * @param Size 接收数据长度
 */
void USB_ReceiveCallback(uint16_t Size) {
  if (0) // !init_finished
  {
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
    return;
  }

  // 自设双缓冲USB
  USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_Active;
  if (USB0_Manage_Object.Rx_Buffer_Active == USB0_Manage_Object.Rx_Buffer_0) {
    USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_1;
  } else {
    USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
  }

  USB0_Manage_Object.Rx_Time_Stamp = DWT_Get_Timestamp();

  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  if (USB0_Manage_Object.Callback_Function != NULL) {
    USB0_Manage_Object.Callback_Function(USB0_Manage_Object.Rx_Buffer_Ready,
                                         Size);
  }
}
