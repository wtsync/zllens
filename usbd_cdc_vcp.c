/**
  ******************************************************************************
  * @brief   USB virtual ComPort media access layer functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_vcp.h"

/* Private variables ---------------------------------------------------------*/
LINE_CODING linecoding =
{
    115200, /* Baud rate     */
    0x00,   /* Stop bits - 1 */
    0x00,   /* Parity - none */
    0x08    /* Num of bits 8 */
};

USART_InitPara USART_InitStructure;

/* Application buffer which's data will be sent over USB CDC device IN endpoint */
extern uint8_t  APP_DATA_Buffer[];

/* Increment this pointer or roll it back to start pointer when writing received data to the APP_Buffer */
extern uint32_t AppBufInPtr;

/* Private function prototypes ----------------------------------------------- */
static uint16_t  VCP_Init      (uint8_t  Config);
static uint16_t  VCP_DeInit    (void);
static uint16_t  VCP_Ctrl      (uint8_t  Cmd, uint8_t* CmdBuf, uint16_t Len);
static uint16_t  VCP_DataTx    (uint8_t* DataBuf, uint16_t Len);
static uint16_t  VCP_DataRx    (uint8_t* DataBuf, uint16_t Len);
static uint16_t  VCP_COMConfig (uint8_t  Config);

CDC_IF_Fop_TypeDef VCP_fops = 
{
    VCP_Init,
    VCP_DeInit,
    VCP_Ctrl,
    VCP_DataTx,
    VCP_DataRx
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the virtual ComPort on the GD32
  * @param  Config: configuration type
  * @retval USBD_OK if configuration type is right, USBD_FAIL else
  */
static uint16_t  VCP_Init (uint8_t Config)
{
    if(Config == DEFAULT_CONFIG)
    {
        NVIC_InitPara NVIC_InitStructure;
  
        /* Usart1 default configuration */
        USART_InitStructure.USART_BRR = 115200;
        USART_InitStructure.USART_WL = USART_WL_8B;
        USART_InitStructure.USART_STBits = USART_STBITS_1;
        USART_InitStructure.USART_Parity = USART_PARITY_RESET;
        USART_InitStructure.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
        USART_InitStructure.USART_RxorTx = USART_RXORTX_RX | USART_RXORTX_TX;

        /* Configure and enable the usart1 */
        GD_EVAL_COMInit(&USART_InitStructure);

        /* Enable the usart1 receive interrupt */
        USART_INT_Set(EVAL_COM1, USART_INT_RBNE, ENABLE);

        /* Configure usart1 receive interrupt */
        NVIC_InitStructure.NVIC_IRQ = EVAL_COM1_IRQn;
        NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
        NVIC_InitStructure.NVIC_IRQSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
  
        return USBD_OK;
    }
    else
    {
        return USBD_FAIL;
    }
}

/**
  * @brief  Deinitializes the virtual ComPort on the GD32
  * @param  None
  * @retval USBD_OK
  */
static uint16_t  VCP_DeInit (void)
{
    return USBD_OK;
}


/**
  * @brief  Handle the CDC device class requests
  * @param  Cmd: command code
  * @param  CmdBuf: command data buffer pointer
  * @param  Len: count of data to be sent (in bytes)
  * @retval USBD_OK
  */
static uint16_t  VCP_Ctrl (uint8_t Cmd, uint8_t* CmdBuf, uint16_t Len)
{ 
    switch (Cmd)
    {
        case SEND_ENCAPSULATED_COMMAND:
            /* No operation for this driver */
            break;

        case GET_ENCAPSULATED_RESPONSE:
            /* No operation for this driver */
            break;

        case SET_COMM_FEATURE:
            /* No operation for this driver */
            break;

        case GET_COMM_FEATURE:
            /* No operation for this driver */
            break;

        case CLEAR_COMM_FEATURE:
            /* No operation for this driver */
            break;

        case SET_LINE_CODING:
            linecoding.dwDTERate = (uint32_t)(CmdBuf[0] | (CmdBuf[1] << 8) | (CmdBuf[2] << 16) | (CmdBuf[3] << 24));
            linecoding.bCharFormat = CmdBuf[4];
            linecoding.bParityType = CmdBuf[5];
            linecoding.bDataBits = CmdBuf[6];
            VCP_COMConfig(SETTING_CONFIG);  /* Set the new configuration */
            break;

        case GET_LINE_CODING:
            CmdBuf[0] = (uint8_t)(linecoding.dwDTERate);
            CmdBuf[1] = (uint8_t)(linecoding.dwDTERate >> 8);
            CmdBuf[2] = (uint8_t)(linecoding.dwDTERate >> 16);
            CmdBuf[3] = (uint8_t)(linecoding.dwDTERate >> 24);
            CmdBuf[4] = linecoding.bCharFormat;
            CmdBuf[5] = linecoding.bParityType;
            CmdBuf[6] = linecoding.bDataBits;
            break;

        case SET_CONTROL_LINE_STATE:
            /* No operation for this driver */
            break;

        case SEND_BREAK:
            /* No operation for this driver */
            break;

        default:
            break;
    }

    return USBD_OK;
}

#define  SIZE_USR_BUF 64
uint8_t  p_sp_buf[SIZE_USR_BUF];
// volatile uint16_t len_usr_buf = 0;
volatile uint16_t offset_head_sp_buf = 0;
volatile uint16_t offset_tail_sp_buf = 0;
uint16_t sp_available(){
    return (offset_tail_sp_buf + SIZE_USR_BUF - offset_head_sp_buf) % SIZE_USR_BUF;
}
uint8_t sp_read(){
    uint8_t tmp;
//    disable_usb_irq();
    tmp = p_sp_buf[offset_head_sp_buf];
    if(offset_head_sp_buf != offset_tail_sp_buf){
        offset_head_sp_buf ++;
        if (SIZE_USR_BUF == offset_head_sp_buf) {
            offset_head_sp_buf = 0;
        }
    }
//    enable_usb_irq();

    return tmp;
}
void sp_2_sp_buf(uint8_t c){
    p_sp_buf[offset_tail_sp_buf] = c;
    offset_tail_sp_buf++;
    if (SIZE_USR_BUF == offset_tail_sp_buf) {
        offset_tail_sp_buf = 0;
    }
    // len_usr_buf = (offset_tail_usr_buf + SIZE_USR_BUF - offset_head_usr_buf) % SIZE_USR_BUF;
}
/**
  * @brief  Real usart received data to be send over virtual usart(USB Tx endpoint)
  * @param  DataBuf: data buffer will be sent
  * @param  Len: count of data to be sent (in bytes)
  * @retval USBD_OK if all operation is right, else USBD_FAIL
  */
static uint16_t  VCP_DataTx (uint8_t* DataBuf, uint16_t Len)
{
    uint8_t dat = 0;
    switch(linecoding.bDataBits)
    {
        case 7:
            dat = USART_DataReceive(EVAL_COM1) & 0x7F;
            break;

        case 8:
            dat = USART_DataReceive(EVAL_COM1);
            break;

        default:
            return USBD_FAIL;
    }
    sp_2_sp_buf(dat);

    return USBD_OK;
}

// #define  SIZE_USR_BUF 64
uint8_t  p_usr_buf[SIZE_USR_BUF];
// volatile uint16_t len_usr_buf = 0;
volatile uint16_t offset_head_usr_buf = 0;
volatile uint16_t offset_tail_usr_buf = 0;

// when call vcp_available, head will NOT be modified
uint16_t vcp_available(){
//    disable_usb_irq();
    // uint16_t tmp = (offset_tail_usr_buf + SIZE_USR_BUF - offset_head_usr_buf) % SIZE_USR_BUF;
    // return tmp;
//    enable_usb_irq();
    return (offset_tail_usr_buf + SIZE_USR_BUF - offset_head_usr_buf) % SIZE_USR_BUF;
}
uint8_t vcp_read(){
    uint8_t tmp;
//    disable_usb_irq();
    tmp = p_usr_buf[offset_head_usr_buf];
    if(offset_head_usr_buf != offset_tail_usr_buf){
        offset_head_usr_buf ++;
        if (SIZE_USR_BUF == offset_head_usr_buf) {
            offset_head_usr_buf = 0;
        }
    }
//    enable_usb_irq();

    return tmp;
}
void vcp_write(uint8_t c){
    APP_DATA_Buffer[AppBufInPtr] = c;
    AppBufInPtr ++;

    /* To avoid buffer overflow */
    if(AppBufInPtr == APP_RX_DATA_SIZE)
    {
        AppBufInPtr = 0;
    }
}
void usb_2_usr_buf(uint8_t c){
    p_usr_buf[offset_tail_usr_buf] = c;
    offset_tail_usr_buf++;
    if (SIZE_USR_BUF == offset_tail_usr_buf) {
        offset_tail_usr_buf = 0;
    }
    // len_usr_buf = (offset_tail_usr_buf + SIZE_USR_BUF - offset_head_usr_buf) % SIZE_USR_BUF;
}

/**
  * @brief  Virtual usart received over USB Rx endpoint are sent over real usart
  * @note   Before exiting this function, the function will block any OUT packet reception on USB endpoint 
  *         If exiting this function before transfer is complete on CDC interface, it will result in 
  *         receiving more data while previous ones are still not sent.
  * @param  Buf: data buffer will be received
  * @param  Len: length of data received (in bytes)
  * @retval USBD_OK
  */
static uint16_t  VCP_DataRx (uint8_t* DataBuf, uint16_t Len)
{
    uint16_t i;
    // uint16_t  tmp;

    for (i = 0; i < Len; i++)
    {
        usb_2_usr_buf(*(DataBuf + i));
        // tmp = *(DataBuf + i);
        // usb_2_usr_buf(tmp);
        // // USART_DataSend(EVAL_COM1, *(DataBuf + i));
        // USART_DataSend(EVAL_COM1, tmp);
        // while(USART_GetBitState(EVAL_COM1, USART_FLAG_TBE) == RESET);
    }

    return USBD_OK;
}
void serial_log(uint8_t* p, uint16_t len){
    int i;
    for (i = 0; i < len; ++i)
    {
        USART_DataSend(EVAL_COM1, p[i]);
        while(USART_GetBitState(EVAL_COM1, USART_FLAG_TBE) == RESET);
    }
}

/**
  * @brief  Configure the ComPort with default values or values received from host
  * @param  Config: can be DEFAULT_CONFIG to have no operation or SETTING_CONFIG 
  *                 to set a configuration received from the host
  * @retval USBD_FAIL if received command data have errors, or will be USBD_OK 
  */
static uint16_t  VCP_COMConfig (uint8_t Config)
{
    if (Config == SETTING_CONFIG)
    {
        uint32_t USART_STBits = USART_InitStructure.USART_STBits;
        uint32_t USART_Parity = USART_InitStructure.USART_Parity;

        /* Set the char format of ComPort */
        switch (linecoding.bCharFormat)
        {
            case 0:
                USART_InitStructure.USART_STBits = USART_STBITS_1;
                break;

            case 1:
                USART_InitStructure.USART_STBits = USART_STBITS_1_5;
                break;

            case 2:
                 USART_InitStructure.USART_STBits = USART_STBITS_2;
                 break;

            default:
                return (USBD_FAIL);
        }

        /* Set the parity type of ComPort */
        switch (linecoding.bParityType)
        {
            case 0:
                USART_InitStructure.USART_Parity = USART_PARITY_RESET;
                break;

            case 1:
                USART_InitStructure.USART_Parity = USART_PARITY_SETEVEN;
                break;

            case 2:
                USART_InitStructure.USART_Parity = USART_PARITY_SETODD;
                break;

            default:
                USART_InitStructure.USART_STBits = USART_STBits;
                return (USBD_FAIL);
        }

        /* Set the data bits: only 8bits and 9bits is supported */
        switch (linecoding.bDataBits)
        {
            case 0x07:
                /* With this configuration a parity (Even or Odd) should be set */
                USART_InitStructure.USART_WL = USART_WL_8B;
                break;

            case 0x08:
                if (USART_InitStructure.USART_Parity == USART_PARITY_RESET)
                {
                    USART_InitStructure.USART_WL = USART_WL_8B;
                }
                else
                {
                    USART_InitStructure.USART_WL = USART_WL_9B;
                }
                break;

            default:
                USART_InitStructure.USART_Parity = USART_Parity;
                USART_InitStructure.USART_STBits = USART_STBits;
                return (USBD_FAIL);
        }

        USART_InitStructure.USART_BRR = linecoding.dwDTERate;
        USART_InitStructure.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
        USART_InitStructure.USART_RxorTx = USART_RXORTX_RX | USART_RXORTX_TX;

        /* Configure and enable the USART */
        GD_EVAL_COMInit(&USART_InitStructure);
    }
    return USBD_OK;
}

/**
  * @brief  Usart intterupt handler
  * @param  None.
  * @retval None.
  */
void  GDEVAL_COM_IRQHandler (void)
{
    if (USART_GetIntBitState(EVAL_COM1, USART_INT_RBNE) != RESET)
    {
        /* Virtual Comport send the received data to the PC Host */
        VCP_DataTx(0,0);
    }

    /* If happens data overrun, clear the ORE flag and recover communication */
    if (USART_GetBitState(EVAL_COM1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_DataReceive(EVAL_COM1);
    }
}
