/**
  ******************************************************************************
  * @brief   USB device driver basic configuration
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_CONF_H
#define __GD32F1X0_USB_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"
#include "gd32150r_eval.h"

/* Exported constants --------------------------------------------------------*/
/* Define if low power mode is enabled; it allows entering the device into DEEP_SLEEP mode
   following USB suspend event and wakes up after the USB wakeup event is received. */
/* #define USB_DEVICE_LOW_PWR_MODE_SUPPORT */

/* Endpoint count used by the CDC device */
#define EP_COUNT          (4)

/* Base address of the allocation buffer, used for buffer descriptor table and packet memory */
#define BUFFER_ADDRESS    (0x0000)

/* Endpoint0, Rx/Tx buffers address */
#define EP0_RX_ADDRESS    (0x40)
#define EP0_TX_ADDRESS    (0x80)

/* Virtual ComPort data Tx buffer address */
#define BULK_TX_ADDRESS   (0xC0) 

/* Virtual ComPort data Rx buffer address */
#define BULK_RX_ADDRESS   (0x110)

/* Virtual ComPort command Tx buffer address */
#define INT_TX_ADDRESS    (0x100)

#endif /* __GD32F1X0_USB_CONF_H */
