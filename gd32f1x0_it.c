/**
  ******************************************************************************
  * @brief   Main interrupt service routines
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_it.h"

/* Private function prototypes -----------------------------------------------*/
void TimingDelay_Decrement(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void  NMI_Handler (void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void  HardFault_Handler (void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void  SVC_Handler (void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void  PendSV_Handler (void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void  SysTick_Handler (void)
{
    TimingDelay_Decrement();
}

/**
  * @brief  This function handles USB FS Handler.
  * @param  None
  * @retval None
  */
void  USB_LP_IRQHandler (void)
{
    USB_Ifr();
}

#ifdef USB_DEVICE_LOW_PWR_MODE_SUPPORT

/**
  * @brief  This function handles USB WakeUp interrupt request.
  * @param  None
  * @retval None
  */
void  USBWakeUp_IRQHandler (void)
{
    EXTI_ClearINTBitState(EXTI_LINE18);
}

#endif
