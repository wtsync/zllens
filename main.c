/**
  ******************************************************************************
  * @brief   Main routine
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
#include "usbd_user.h"
#include "arduino_compact.h"

/* Private variables ---------------------------------------------------------*/
USB_DEVICE_HANDLE  USB_Device_dev;

/* Private functions ---------------------------------------------------------*/
__IO uint32_t TimingDelay = 0;
__IO uint32_t TimeAfterBoot = 0;

/* Private function prototypes -----------------------------------------------*/
void delay_ms_not_recommand(__IO uint32_t nTime);
void SysTick_Configuration(void);
void GPIO_Configuration(void);
void TimingDelay_Decrement(void);

extern void    push_usr_buf(uint8_t c);
extern uint8_t pop_usr_buf(void);
extern bool    available_usr_buf(void);

extern uint16_t vcp_available(void);
extern uint8_t  vcp_read(void);
extern void     vcp_write(uint8_t);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configure the GPIO ports
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
    /* Enable the GPIO_LED Clock */
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOB, ENABLE);
}

extern int digitalRead(int pin) ;
extern void digitalWrite(int pin, int val);
extern void pinMode(int pin, int mod) ;

void compact_arduino() {
    GPIO_Configuration();
    SysTick_Configuration();
}


extern void setup(void);
extern void loop(void);
extern void loop2(void);
// void setup() {
//     pinMode(0, OUTPUT);
// }

// uint8_t dat = 0;
// int step = 0;
// void loop() {
//     while(vcp_available())
//     {
//         dat = vcp_read();
//         vcp_write(dat);
//     }

//     if(TimingDelay){
//         return;
//     }
//     if (1 == step)
//     {
//         TimingDelay = 1000;
//         digitalWrite(0, HIGH);
//         step = 2;
//     }else
//     {
//         TimingDelay = 300;
//         digitalWrite(0, LOW);
//         step = 1;
//     }
// }

/**
  * @brief  Main routine will construct a USB virtual ComPort device
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Hardware platform initialization */
    USB_HWP_Init();

    /* pull down DP 15kOhm pull up resistor*/
    // USBD_Disconnect(&USB_Device_dev);
    // USB_Cable_Config(DISABLE);

    /* Configure usb clock */
    USB_HWP_ClockConfig();

    /* Configure USB interrupt */
    USB_HWP_USBINTConfig();

    /* USB device configuration */
    USBD_Init(&USB_Device_dev,
              &USER_desc,
              &USBD_CDC_cb,
              &USER_cb);

    /* Connect the USB device */
    USBD_Connect(&USB_Device_dev);

    compact_arduino();
    setup();
    while (1) {
        loop();
    }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay_ms_not_recommand(__IO uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0);
}

uint32_t getTimeAfterBoot(){
  return TimeAfterBoot;
}
void busy_delay_ms(__IO uint32_t n){
  uint32_t last = getTimeAfterBoot();
  while(((getTimeAfterBoot() + 0x10000 - last) % 0x10000) < n);
}

/**
  * it system tick interrupt, look like 1 ms. 
  */
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    {
        TimingDelay--;
    }
    TimeAfterBoot ++;
}

