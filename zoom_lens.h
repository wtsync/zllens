/*
 * for arduino 
 */
#ifndef __ZOOM_LENS_H
#define __ZOOM_LENS_H

#include "stdint.h"

// #define _DEBUG_COMMOND_
#define _NOTIFY_MOVE_DONE_


//#define uint8_t unsigned char
//#define uint32_t unsigned int


#define HIGH 1
#define LOW  0

#define INPUT 1
#define INPUT_PULLUP 2
#define OUTPUT 3

#ifndef bool
#define bool  _Bool
#define true  1
#define false 0
#endif

// NACK
#define NACK_SUM                            1
#define NACK_CMD_NOT_FOUND                  2
#define NACK_PARAM_COUNT                    3
#define NACK_PARAM_OUT_OF_RANGE             4
#define NACK_SELF_TEST_FAIL                 5
#define NACK_IAP_PINS_TYPE                  6
#define NACK_IAP_ZOOM_AF_TABLE_SIZE         7
#define NACK_IAP_ZOOM_AF_TABLE_BIGGER       8

// self test fail detail
#define STF_ZOOM_NO_BEYOND      1
#define STF_ZOOM_NO_

// const nack_min = 1;
// const nack_max = 8;
// const nack_param_count_table[9] = {0,0,0,0,0,     0,0,0,1}

// ACK
#define ACK_DEBUG               0
#define ACK_VERSION_GET         1
#define ACK_STEP_GET            2
#define ACK_ZOOM_GET            3
#define ACK_NOTIFY              4
#define ACK_ZERO_PHASE_GET      5

// NOTIFY value
#define NOTOFY_MOVE_DONE        1
#define NOTIFY_KEY_ZOOM_ADD     2
#define NOTIFY_KEY_ZOOM_SUB     3
#define NOTIFY_KEY_FOCUS_ADD    4
#define NOTIFY_KEY_FOCUS_SUB    5
#define NOTIFY_KEY_AUTO_FOCUS   6
// #define NOTIFY_KEY_CAPTURE      7


// commands

#define VERSION_GET                     1   
#define STEP_GET                        2
#define STEP_SET                        3
#define CONFIG_PINS_TYPE                4
#define CONFIG_ZOOM_MAX                 5
#define CONFIG_ZOOM_AF_TABLE            6
#define SELF_TEST_BACK_TO_LAST_POS      7
#define ZOOM_IN                         8
#define ZOOM_OUT                        9
#define ZOOM_GET                        10
#define ZERO_PHASE_GET                  11
#define CONFIG_ERASE_ALL                12

const int cmd_max = 12;
const int cmd_min = 1;
const int cmd_param_count_table[13] = {0,    0,0,4,1,1,   5,0,0,0,0, 0, 0};

// CONFIG_PINS_TYPE
#define KL501_TYPE  1
#define T5182_TYPE   2
// zoom AP, AM, BP, BM, ZD, focus AP, AM, BP, BM, ZD
const uint8_t KL501_pins[10] =  {5,6,7,16,14, 1,2,3 ,4,15};
const uint8_t T5182_pins[10]  = {4,2,1,3 ,15, 6,5,16,7,14}; // 16 ==> PB0
const uint8_t zoom_ng_pin = 10;
const uint8_t focus_ng_pin = 11;
const uint8_t driver_zoom_enable = 0;
const uint8_t driver_focus_enable = 17;

// zoom min, max, focus min, max
// min is negative, means that time zero detect output LOW !!!
const int KL501_cmd_step_max[4] = {-606,594,-505,457};
//const int T5182_cmd_step_max[4]  = { -57,658, -70,245};
const int T5182_cmd_step_max[4]  = { -80,700, -100,400};

// self_test max range is bigger than KL501_cmd_step_max[?],T5182_cmd_step_max[?]
#define SELF_TEST_RETRY_MAX     1000
#define SELF_TEST_ADD_RETRY_MAX 100
#define SELF_TEST_SUB_RETRY_MAX 20

//// from output LOW --> output HIGH
//const uint8_t KL501_ZOOM_phases[4][4] = {
//    {0,1,1,0},
//    {1,0,1,0},
//    {1,0,0,1},
//    {0,1,0,1}
//}
//// from output LOW --> output HIGH
//const uint8_t KL501_FOCUS_phases[4][4] = {
//    {0,1,1,0},
//    {1,0,1,0},
//    {1,0,0,1},
//    {0,1,0,1}
//}

//// from output LOW --> output HIGH
//const uint8_t T5182_ZOOM_phases[4][4] = {
//    {0,1,1,0},
//    {1,0,1,0},
//    {1,0,0,1},
//    {0,1,0,1}
//}
//// from output LOW --> output HIGH
//const uint8_t T5182_FOCUS_phases[4][4] = {
//    {0,1,1,0},
//    {1,0,1,0},
//    {1,0,0,1},
//    {0,1,0,1}
//}
// we found all these steppers, phase 0 --> phase 3 all same.
const uint8_t phase_values[][4] = {
    // phase 0 --> AP 0, BP 1
    {0,1,1,0},
    // phase 1 --> AP 0, BP 0
    {0,1,0,1},
    // phase 2 --> AP 1, BP 0
    {1,0,0,1},
    // phase 2 --> AP 1, BP 1
    {1,0,1,0}
};

// in fact, AF_TABLE is start from BASE+8
// #define IAP_ADDR_PAGE            0x16
#define GD32F150G8
// GD32F150G4, has 16 page flash(16k)
// GD32F150G6, has 32 page flash(32k)
// GD32F150G8, has 64 page flash(64k)
#ifdef GD32F150G8
#define IAP_ADDR_TYPE               0x0800FC00
#define IAP_ADDR_AF_TABLE_SIZE      0x0800FC04
#define MAX_IAP_AF_TABLE_ITEMS      0x20
#define IAP_ADDR_AF_TABLE_BASE      0x0800FC08
#else
#ifdef GD32F150G6
#define IAP_ADDR_TYPE               0x08007C00
#define IAP_ADDR_AF_TABLE_CURR_SIZE 0x08007C04
#define MAX_IAP_AF_TABLE_ITEMS      0x20
#define IAP_ADDR_AF_TABLE_BASE      0x08007C08
#else
#ifdef GD32F150G4
#define IAP_ADDR_TYPE               0x08003C00
#define IAP_ADDR_AF_TABLE_CURR_SIZE 0x08003C04
#define MAX_IAP_AF_TABLE_ITEMS      0x20
#define IAP_ADDR_AF_TABLE_BASE      0x08003C08
#endif
#endif
#endif



#endif

