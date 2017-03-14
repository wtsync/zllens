/**
  ******************************************************************************
  * @brief   Interrupt handlers header files
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1x0_IT_H
#define __GD32F1x0_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"

/* Exported functions ------------------------------------------------------- */
void NMI_Handler        (void);
void HardFault_Handler  (void);
void SVC_Handler        (void);
void PendSV_Handler     (void);
void SysTick_Handler    (void);
void USB_LP_IRQHandler  (void);

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1x0_IT_H */
