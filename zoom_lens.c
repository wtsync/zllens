#include "zoom_lens.h"
#include "stdint.h"
#include "gd32f1x0.h"
#include "arduino_compact.h"
#include "string.h"

struct STEPPER_TYPE_CONFIG_T {
    uint8_t AP;
    uint8_t AM;
    uint8_t BP;
    uint8_t BM;
    uint8_t ZD;

    int     min_pos;
    int     max_pos;
}type_config_zoom, type_config_focus;

enum SELF_TEST_T {
    E_UNTEST,
    E_NO_START_FROM_LOW,
    E_NO_FIRST_HIGH,
    E_NO_FIRST_STILL_HIGH,
    E_NO_FIRST_LOW,
    E_NO_FIRST_STILL_LOW,
    E_NO_SECOND_HIGH,
    E_SELF_TEST_OK,
};

struct STEPPER_STATUS_T {
    int cur_pos;
    int pos_should_be;
    int phase_when_step_0;
    // phase = (step + phase_when_step_0) % 4
    enum SELF_TEST_T self_tested;
}status_zoom, status_focus;

enum IAP_LOAD_ERROR_T {
    E_OK,
    E_TYPE_ERR,
    E_AF_TABLE_SIZE_ERR,
    E_AF_TABLE_BIGGER_ERR
}iap_load_success;

int zoom_default_pos = 0;
int focus_default_pos = 0;

#define _MAX_PAYLOAD_LEN_ 17
#define _MAX_PACKAGE_LEN_ 20
unsigned char g_buf[_MAX_PACKAGE_LEN_];
int  g_len = 0;



// ======================== function declaration =================
extern void busy_delay_ms(uint32_t n);
extern void pinMode(int pin, int mod);
extern int digitalRead(int pin) ;
extern void digitalWrite(int pin, int val);
// virtual serial port
extern uint16_t vcp_available(void);
extern uint8_t vcp_read(void);
extern void vcp_write(uint8_t c);
// serial port
extern uint16_t sp_available(void);
extern uint8_t sp_read(void);
//
void process_payload_after_check_nack(unsigned char* p, int len);

// ======================== function defination  =================
void stepper_do_phase(struct STEPPER_TYPE_CONFIG_T * p_config, int phase){
    phase %= 4;
    phase += 4;
    phase %= 4;

    digitalWrite(p_config->AP, phase_values[phase][0]);
    digitalWrite(p_config->AM, phase_values[phase][1]);
    digitalWrite(p_config->BP, phase_values[phase][2]);
    digitalWrite(p_config->BM, phase_values[phase][3]);
}
void busy_wait_after_phase(){
    busy_delay_ms(5);
}

// if now is WANT, return 0
// if move N steps, get WANT, return N, N is 1 to step_over
// if move step_over, no WANT, return -1
int find_then_ret_steps(bool is_add, int high_low, int step_over,
                   struct STEPPER_TYPE_CONFIG_T* p_config, struct STEPPER_STATUS_T* p_status){
    int i = 0;
    if(high_low == digitalRead(p_config->ZD)){
        return 0;
    }

    for(i = 1; i <= step_over; i++){
        if(is_add){
            p_status->cur_pos ++;
        }else{
            p_status->cur_pos--;
        }
        stepper_do_phase(p_config, p_status->cur_pos + p_status->phase_when_step_0);
        busy_wait_after_phase();
        if(1 == i){
            busy_wait_after_phase();
        }

        // have moved i steps.
        if(high_low == digitalRead(p_config->ZD)){
            return i;
        }
    }
    return -1;
}

void stepper_self_test(struct STEPPER_TYPE_CONFIG_T* p_config, struct STEPPER_STATUS_T* p_status, int end_pos){
    // step 1
    // after power on, p_status->phase will be massive, move some steps first
    p_status->cur_pos = 0;
    p_status->phase_when_step_0 = 0;
    stepper_do_phase(p_config, p_status->cur_pos + p_status->phase_when_step_0);
    busy_wait_after_phase();
    if(HIGH == digitalRead(p_config->ZD)){
        find_then_ret_steps(false, LOW, 1, p_config, p_status);
    }else{
        find_then_ret_steps(true, HIGH, 1, p_config, p_status);
    }

    // step 2
    // if HIGH, goto LOW, then we can find "first 0-->1 beyond", not "1-->0 beyond"
    if (HIGH == digitalRead(p_config->ZD)) {
        if(find_then_ret_steps(false, LOW, SELF_TEST_RETRY_MAX, p_config, p_status) < 0){
            p_status->self_tested = E_NO_START_FROM_LOW;
            return;
        }
    }

    // step 3: find 0 --> 1, then mark this position is 1
    if(find_then_ret_steps(true, HIGH, SELF_TEST_RETRY_MAX, p_config, p_status) < 0){
        p_status->self_tested = E_NO_FIRST_HIGH;
        return;
    }

    p_status->phase_when_step_0 = (p_status->cur_pos + p_status->phase_when_step_0 - 1) % 4;
    p_status->cur_pos = 1;

    // til step 3, we can say "test OK"
//    p_status->pos_should_be = end_pos;
//    p_status->self_tested = E_SELF_TEST_OK;
//    return ;

    // we do something more

    // step 4.1
    // now is HIGH, plus 100(self_test_add_range), sub 100, should be HIGH
    // true + LOW, means no test
    if(find_then_ret_steps(true, LOW, SELF_TEST_ADD_RETRY_MAX, p_config, p_status) >= 0){
        p_status->self_tested = E_NO_FIRST_STILL_HIGH;
        return;
    }
    if(find_then_ret_steps(false, LOW, SELF_TEST_ADD_RETRY_MAX, p_config, p_status) >= 0){
        p_status->self_tested = E_NO_FIRST_STILL_HIGH;
        return;
    }

    // step 4.2
    // now is HIGH, sub less than 5 steps, should be LOW
    // 5 steps is stepper not tie good
    if(find_then_ret_steps(false, LOW, 5, p_config, p_status) < 0){
        p_status->self_tested = E_NO_FIRST_LOW;
        return;
    }

    // step 4.3
    // now is LOW, sub 20(self_test_sub_range), plus 20, should be 0
    if(find_then_ret_steps(false, HIGH, SELF_TEST_SUB_RETRY_MAX, p_config, p_status) >= 0){
        p_status->self_tested = E_NO_FIRST_STILL_LOW;
        return;
    }
    if(find_then_ret_steps(true, HIGH, SELF_TEST_SUB_RETRY_MAX, p_config, p_status) >= 0){
        p_status->self_tested = E_NO_FIRST_STILL_LOW;
        return;
    }
    // step 4.4
    // now is 1-->0, plus less than 5 steps, should be 1
    if(find_then_ret_steps(true, HIGH, 5, p_config, p_status) < 0){
        p_status->self_tested = E_NO_SECOND_HIGH;
        return;
    }

    p_status->pos_should_be = end_pos;
    p_status->self_tested = E_SELF_TEST_OK;
    return ;
}

//FMC_State FMC_ErasePage(uint32_t Page_Address);
void IAP_clear_all(){
    FMC_Unlock();
    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );
    FMC_ErasePage(IAP_ADDR_TYPE);
    FMC_Lock();
}

// #define IAP_set FMC_ProgramWord
void IAP_set(uint32_t addr, uint32_t data){
    FMC_Unlock();
    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );
    FMC_ProgramWord(addr, data);
    FMC_Lock();
}
// void IAP_set(uint32_t addr_offset, uint32_t data){
//     ;
// }
uint32_t IAP_get(uint32_t addr){
    return (*(__IO uint32_t*)addr);
}

bool type_to_type_configs(uint8_t type, struct STEPPER_TYPE_CONFIG_T* zoom, struct STEPPER_TYPE_CONFIG_T* focus){
    if(KL501_TYPE == type){
        zoom->AP = KL501_pins[0];
        zoom->AM = KL501_pins[1];
        zoom->BP = KL501_pins[2];
        zoom->BM = KL501_pins[3];
        zoom->ZD = KL501_pins[4];
        zoom->min_pos = KL501_cmd_step_max[0] + 5;
        zoom->max_pos = KL501_cmd_step_max[1] - 5;
        focus->AP = KL501_pins[5];
        focus->AM = KL501_pins[6];
        focus->BP = KL501_pins[7];
        focus->BM = KL501_pins[8];
        focus->ZD = KL501_pins[9];
        focus->min_pos = KL501_cmd_step_max[2] + 5;
        focus->max_pos = KL501_cmd_step_max[3] - 5;
        return true;
    }else if(T5182_TYPE == type){
        zoom->AP = T5182_pins[0];
        zoom->AM = T5182_pins[1];
        zoom->BP = T5182_pins[2];
        zoom->BM = T5182_pins[3];
        zoom->ZD = T5182_pins[4];
        zoom->min_pos = T5182_cmd_step_max[0] + 5;
        zoom->max_pos = T5182_cmd_step_max[1] - 5;
        focus->AP = T5182_pins[5];
        focus->AM = T5182_pins[6];
        focus->BP = T5182_pins[7];
        focus->BM = T5182_pins[8];
        focus->ZD = T5182_pins[9];
        focus->min_pos = T5182_cmd_step_max[2] + 5;
        focus->max_pos = T5182_cmd_step_max[3] - 5;
        return true;
    }else{
        return false;
    }
}

//bool check_package_sum(unsigned char* p, u8 len){
//    int i = 0;
//    unsigned char sum;

//    sum = p[len - 1];
//    for(i = len - 2; i >= 0; i++){
//        sum -= p[i];
//    }
//    return 0 == sum;
//}

void load_IAP_configs(){
    int i;
    unsigned char tmp;

    // type
    tmp = IAP_get(IAP_ADDR_TYPE) & 0xFF;
    if((1 != tmp) && (2 != tmp)){
        iap_load_success = E_TYPE_ERR;
        return;
    }
    type_to_type_configs(tmp, &type_config_zoom, &type_config_focus);

    // af table size
    tmp = IAP_get(IAP_ADDR_AF_TABLE_SIZE) & 0xFF;
    if((tmp <= 0) || (tmp >= 30)){
        iap_load_success = E_AF_TABLE_SIZE_ERR;
        return;
    }

    // af table zoom stepper position, should be bigger to litter
    for(i = 1; i < tmp; i++){
        if(IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i) <= IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i + 8)){
            iap_load_success = E_AF_TABLE_BIGGER_ERR;
            return;
        }
    }

    zoom_default_pos = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8);
    focus_default_pos = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 + 4);

    iap_load_success = E_OK;
    return;
}

void setup() {
    // no on board leds
//    digitalWrite(pin_led, LOW);
//    pinMode(pin_led, OUTPUT);
//    digitalWrite(pin_led, LOW);
    load_IAP_configs();

    if(E_OK == iap_load_success){
        // disable driver chip first ==================================================
        digitalWrite(zoom_ng_pin, HIGH);
        pinMode(zoom_ng_pin, OUTPUT);
        digitalWrite(zoom_ng_pin, HIGH);

        digitalWrite(focus_ng_pin, HIGH);
        pinMode(focus_ng_pin, OUTPUT);
        digitalWrite(focus_ng_pin, HIGH);

        // ==================================================
        // init phase initial position detection
        pinMode(type_config_zoom.ZD, INPUT);
        pinMode(type_config_focus.ZD, INPUT);
        // A B pins
        pinMode(type_config_zoom.AP, OUTPUT);
        pinMode(type_config_zoom.AM, OUTPUT);
        pinMode(type_config_zoom.BP, OUTPUT);
        pinMode(type_config_zoom.BM, OUTPUT);
        pinMode(type_config_focus.AP, OUTPUT);
        pinMode(type_config_focus.AM, OUTPUT);
        pinMode(type_config_focus.BP, OUTPUT);
        pinMode(type_config_focus.BM, OUTPUT);

    //    pinMode(pin_button, INPUT);
    //    last_button_pin_state = digitalRead(pin_button);

        // enable driver chip ===============================
        digitalWrite(zoom_ng_pin, LOW);
        digitalWrite(focus_ng_pin, LOW);
        // ==================================================
    }

    // 74HC244 enable
    {
        pinMode(driver_zoom_enable, OUTPUT);
        pinMode(driver_focus_enable, OUTPUT);
        digitalWrite(driver_zoom_enable, LOW);
        digitalWrite(driver_focus_enable, LOW);
    }

#ifdef _DEBUG_COMMOND_
    status_zoom.cur_pos = 0;
    status_focus.cur_pos = 0;
#else
    if(E_OK == iap_load_success){
        stepper_self_test(&type_config_zoom, &status_zoom, zoom_default_pos);
        stepper_self_test(&type_config_focus, &status_focus, focus_default_pos);
    }else{
        status_focus.self_tested = E_UNTEST;
        status_zoom.self_tested = E_UNTEST;
    }
#endif

    // init g_buf =======================================
    g_len = 0;
    // ==================================================
}

void push_char_pop_package(unsigned char c, unsigned char* p, int* len, int size_p) {
    // push
    (*len) = 0;
    g_buf[g_len] = c;
    g_len ++;

    // state: wait header, wait len, wait command and param, wait sum
    if (1 == g_len) {
        if (0x11 != g_buf[0]) {
            g_len = 0;
        }
    } else if (2 == g_len) {
        // payload length = g_buf[1]
        if ((g_buf[1] + 3) >= _MAX_PACKAGE_LEN_) {
            g_len = 0;
        }
    } else {
        if ((g_buf[1] + 3) == g_len) {
            if(size_p < g_len){
                g_len = 0;
            }
            memcpy(p, g_buf, g_len);
            (*len) = g_len;

            g_len = 0;
        } else if ((g_buf[1] + 3) < g_len) {
            // something wrong
            g_len = 0;
        }else{
            // need more
        }
    }
}

void vcp_write_payload(unsigned char * p, int len){
    int i;
    unsigned char sum = 0;

    vcp_write(0x11);        sum += 0x11;
    vcp_write(len);         sum += len;
    for(i = 0; i < len; i++){
        vcp_write(p[i]);    sum += p[i];
    }
    vcp_write(sum);
}
int param_HL_to_int(unsigned char h, unsigned char l){
    int tmp;
    tmp = h & 0x7F;
    tmp *= 0x100;
    tmp += l;

    if(h & 0x80){
        return -tmp;
    }else{
        return tmp;
    }
}
unsigned char param_int_to_H(int n){
    if((n > 0x7FFF) || (n < (-0x7FFF))){
        return 0;
    }
    if(n >= 0){
        return n >> 8;
    }else{
        return (((-n) >> 8) + 0x80);
    }
}
unsigned char param_int_to_L(int n){
    if((n > 0x7FFF) || (n < -0x7FFF)){
        return 0;
    }
    if(n >= 0){
        return n % 0x100;
    }else{
        return (-n) % 0x100;
    }
}

// assume payload length is OK
bool payload_param_limit_check_ok(unsigned char* p, int len){
    unsigned char cmd;
    int tmp;
    unsigned char nack_str[3];

    cmd = p[0];
    if(STEP_SET == cmd){
        // command: payload[0]
        // zoom stepper: payload[1] [2]
        // focus stepper: payload[3] [4]
        tmp = param_HL_to_int(p[1], p[2]);
        if((tmp < type_config_zoom.min_pos) || (tmp > type_config_zoom.max_pos)){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_PARAM_OUT_OF_RANGE;
            vcp_write_payload(nack_str, 2);
            return false;
        }

        tmp = param_HL_to_int(p[3], p[4]);
        if((tmp < type_config_focus.min_pos) || (tmp > type_config_focus.max_pos)){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_PARAM_OUT_OF_RANGE;
            vcp_write_payload(nack_str, 2);
            return false;
        }
    }else if(CONFIG_PINS_TYPE == cmd){
        if((1 != p[1]) && (2 != p[1])){
            return false;
        }
    }else if(CONFIG_ZOOM_MAX == cmd){
        // 1 to MAX_IAP_AF_TABLE_ITEMS
        return (p[1] > 0) && (p[1] <= MAX_IAP_AF_TABLE_ITEMS);
    }else if(CONFIG_ZOOM_AF_TABLE == cmd){
        // command: payload[0]
        // zoom value: payload[1]
        // zoom stepper: payload[2] [3]
        // focus stepper: payload[4] [5]
        tmp = param_HL_to_int(0, p[1]);
        if((tmp <= 0) || (tmp > MAX_IAP_AF_TABLE_ITEMS)){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_PARAM_OUT_OF_RANGE;
            vcp_write_payload(nack_str, 2);
            return false;
        }

        tmp = param_HL_to_int(p[2], p[3]);
        if((tmp < type_config_zoom.min_pos) || (tmp > type_config_zoom.max_pos)){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_PARAM_OUT_OF_RANGE;
            vcp_write_payload(nack_str, 2);
            return false;
        }

        tmp = param_HL_to_int(p[4], p[5]);
        if((tmp < type_config_focus.min_pos) || (tmp > type_config_focus.max_pos)){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_PARAM_OUT_OF_RANGE;
            vcp_write_payload(nack_str, 2);
            return false;
        }
    }

    return true;
}

void process_package_nack_ack_include(unsigned char *p, int len){
    int i;
    unsigned char sum = 0;
    unsigned char nack_str[_MAX_PAYLOAD_LEN_];
    int payload_len = len - 3;
    unsigned char* payload_p = p + 2;
    unsigned char cmd = p[2];
    int param_len = len - 4;

    if(len <= 3){
        // payload is nothing, vcp write nothing
        return;
    }

    // nacks.sum
    for(i = 0; (i + 1) < len; i++){
        sum += p[i];
    }
    if(sum != p[len - 1]){
        nack_str[0] = 0x0A;
        nack_str[1] = NACK_SUM;
        vcp_write_payload(nack_str, 2);
        return ;
    }
    //nacks.unknown command
    if((cmd < cmd_min) || (cmd > cmd_max)){
        nack_str[0] = 0x0A;
        nack_str[1] = NACK_CMD_NOT_FOUND;
        vcp_write_payload(nack_str, 2);
        return;
    }
    // nacks.param count
    if(cmd_param_count_table[cmd] != param_len){
        nack_str[0] = 0x0A;
        nack_str[1] = NACK_PARAM_COUNT;
        vcp_write_payload(nack_str, 2);
        return;
    }
    // nack.param out of range
    if(false == payload_param_limit_check_ok(payload_p, payload_len)){
        return;
    }
    // nack.self test
    // nack.iap config
    // factory config command, not check self test.
//    if((VERSION_GET                     == cmd) ||
//       (STEP_GET                        == cmd) ||
//       (STEP_SET                        == cmd) ||
//       (SELF_TEST_BACK_TO_LAST_POS      == cmd) ||
//       (ZOOM_IN                         == cmd) ||
//       (ZOOM_OUT                        == cmd) ||
//       (ZOOM_GET                        == cmd) ||
//       ){}
    if((CONFIG_PINS_TYPE != cmd) &&
       (CONFIG_ZOOM_AF_TABLE != cmd) &&
       (SELF_TEST_BACK_TO_LAST_POS != cmd) &&
       (CONFIG_ZOOM_MAX != cmd) &&
       (ZERO_PHASE_GET != cmd) &&
       (CONFIG_ERASE_ALL != cmd)){
        // nack.self tesst
        if(E_TYPE_ERR == iap_load_success){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_IAP_PINS_TYPE;
            vcp_write_payload(nack_str, 2);
            return;
        }else if(E_AF_TABLE_SIZE_ERR == iap_load_success){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_IAP_ZOOM_AF_TABLE_SIZE;
            vcp_write_payload(nack_str, 2);
            return;
        }else if(E_AF_TABLE_BIGGER_ERR == iap_load_success){
            nack_str[0] = 0x0A;
            nack_str[1] = NACK_IAP_ZOOM_AF_TABLE_BIGGER;
            vcp_write_payload(nack_str, 2);
            return;
        }else{
            // assume iap_load_success = E_OK
            if(E_SELF_TEST_OK != status_zoom.self_tested){
                nack_str[0] = 0x0A;
                nack_str[1] = NACK_SELF_TEST_FAIL;
                nack_str[2] = 1;
                nack_str[3] = status_zoom.self_tested;
                // nack_str[4] = type_config_zoom.ZD;
                // nack_str[5] = digitalRead(type_config_zoom.ZD);
                // vcp_write_payload(nack_str, 6);
                vcp_write_payload(nack_str, 4);
                return;
            }
            if(E_SELF_TEST_OK != status_focus.self_tested){
                nack_str[0] = 0x0A;
                nack_str[1] = NACK_SELF_TEST_FAIL;
                nack_str[2] = 2;
                nack_str[3] = status_focus.self_tested;
                // nack_str[4] = type_config_focus.ZD;
                // nack_str[5] = digitalRead(type_config_focus.ZD);
                // vcp_write_payload(nack_str, 6);
                vcp_write_payload(nack_str, 4);
                return;
            }
        }
    }

    // now it's time for ACK
    process_payload_after_check_nack(payload_p, payload_len);
}

void process_payload_after_check_nack(unsigned char* p, int len){
    unsigned char cmd = p[0];
    int af_table_size = 0;
    int i = 0;
    int tmp;
    unsigned char ack[0x10];
    if(VERSION_GET == cmd){
        ack[0] = 0xAC;
        ack[1] = ACK_VERSION_GET;
        ack[2] = 0x11;
        ack[3] = 0x22;
        ack[4] = 0x33;
        ack[5] = 0x44;
        ack[6] = 0x55;
        ack[7] = 0x66;
        ack[8] = 0x77;
        ack[9] = 0x88;
        vcp_write_payload(ack, 10);
    }else if(STEP_GET == cmd){
        ack[0] = 0xAC;
        ack[1] = ACK_STEP_GET;
        ack[2] = param_int_to_H(status_zoom.pos_should_be);
        ack[3] = param_int_to_L(status_zoom.pos_should_be);
        ack[4] = param_int_to_H(status_focus.pos_should_be);
        ack[5] = param_int_to_L(status_focus.pos_should_be);
        vcp_write_payload(ack, 6);
    }else if(STEP_SET == cmd){
        status_zoom.pos_should_be  = param_HL_to_int(p[1], p[2]);
        status_focus.pos_should_be = param_HL_to_int(p[3], p[4]);

        ack[0] = 0xAC;
        vcp_write_payload(ack, 1);
    }else if(CONFIG_PINS_TYPE == cmd){
        IAP_set(IAP_ADDR_TYPE, p[1]);

        ack[0] = 0xAC;
        ack[1] = ACK_DEBUG;
        ack[2] = IAP_get(IAP_ADDR_TYPE) & 0xFF;
        vcp_write_payload(ack, 3);

        // try load IAP again.
        load_IAP_configs();
    }else if(CONFIG_ZOOM_MAX == cmd){
        IAP_set(IAP_ADDR_AF_TABLE_SIZE, (p[1]));

        ack[0] = 0xAC;
        ack[1] = ACK_DEBUG;
        ack[2] = IAP_get(IAP_ADDR_AF_TABLE_SIZE) & 0xFF;
        vcp_write_payload(ack, 3);

        // try load IAP again.
        load_IAP_configs();
    }else if(CONFIG_ZOOM_AF_TABLE == cmd){
//        p[1] < MAX_IAP_AF_TABLE_ITEMS
        IAP_set(IAP_ADDR_AF_TABLE_BASE + 8 * p[1],
                param_HL_to_int(p[2], p[3]));
        IAP_set(IAP_ADDR_AF_TABLE_BASE + 8 * p[1] + 4,
                param_HL_to_int(p[4], p[5]));

        tmp = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * p[1]);
        ack[0] = 0xAC;
        ack[1] = ACK_DEBUG;
        ack[2] = param_int_to_H(tmp);
        ack[3] = param_int_to_L(tmp);
        tmp = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * p[1] + 4);
        ack[4] = param_int_to_H(tmp);
        ack[5] = param_int_to_L(tmp);
        vcp_write_payload(ack, 6);

        // try load IAP again.
        load_IAP_configs();
    }else if(SELF_TEST_BACK_TO_LAST_POS == cmd){
        // TODO
        ack[0] = 0xAC;
        vcp_write_payload(ack, 1);

        stepper_self_test(&type_config_zoom, &status_zoom, zoom_default_pos);
        stepper_self_test(&type_config_focus, &status_focus, focus_default_pos);
    }else if(ZOOM_IN == cmd){
        // status_zoom.pos_should_be ==> zoom value
        // zoom value ++
        // stepper --
        af_table_size = IAP_get(IAP_ADDR_AF_TABLE_SIZE);
        for(i = 1; i <= af_table_size; i++){
            if(status_zoom.pos_should_be > IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i)){
                status_zoom.pos_should_be  = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i);
                status_focus.pos_should_be = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i + 4);
                break;
            }
        }
        ack[0] = 0xAC;
        vcp_write_payload(ack, 1);
    }else if(ZOOM_OUT == cmd){
        // status_zoom.pos_should_be ==> zoom value
        af_table_size = IAP_get(IAP_ADDR_AF_TABLE_SIZE);
        for(i = af_table_size; i >= 1; i--){
            if(status_zoom.pos_should_be < IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i)){
                status_zoom.pos_should_be  = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i);
                status_focus.pos_should_be = IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i + 4);
                break;
            }
        }
        ack[0] = 0xAC;
        vcp_write_payload(ack, 1);
    }else if(ZOOM_GET == cmd){
        af_table_size = IAP_get(IAP_ADDR_AF_TABLE_SIZE);
        // not i == af_table_size
        for(i = 1; i < af_table_size; i++){
            if(status_zoom.pos_should_be >= IAP_get(IAP_ADDR_AF_TABLE_BASE + 8 * i + 8)){
//                ack[2] = i;
//                ack[3] = af_table_size;
                break;
            }
        }
        ack[0] = 0xAC;
        ack[1] = ACK_ZOOM_GET;
        ack[2] = i;
        ack[3] = af_table_size;
        vcp_write_payload(ack, 4);
    }else if(ZERO_PHASE_GET == cmd){
        ack[0] = 0xAC;
        ack[1] = ACK_ZERO_PHASE_GET;
        ack[2] = digitalRead(type_config_zoom.ZD);
        ack[3] = digitalRead(type_config_focus.ZD);
        vcp_write_payload(ack, 4);
    }else if(CONFIG_ERASE_ALL == cmd){
        IAP_clear_all();

        ack[0] = 0xAC;
        vcp_write_payload(ack, 1);
    }
}

bool is_moving = false;
void loop() {
    static int timer_count = 0;
    unsigned char p[_MAX_PACKAGE_LEN_];
    int len;

    // if too many time no data in
    if(timer_count < 100){
        timer_count ++;
    }else{
        g_len = 0;
    }

    // put your main code here, to run repeatedly:
    if (vcp_available()) {
        unsigned char inByte = vcp_read();
        timer_count = 0;
#ifdef _DEBUG_COMMOND_
        if(('+' == inByte) || ('=' == inByte)){
            status_focus.cur_pos += 1;
            status_focus.cur_pos %= 4;
            stepper_do_phase(&type_config_focus, status_focus.cur_pos);
            busy_wait_after_phase();
//            vcp_write_payload(p, len);
        }else if('-' == inByte){
            status_focus.cur_pos += 3;
            status_focus.cur_pos %= 4;
            stepper_do_phase(&type_config_focus, status_focus.cur_pos);
            busy_wait_after_phase();
        }else if('0' == inByte){
            status_zoom.cur_pos += 1;
            status_zoom.cur_pos %= 4;
            stepper_do_phase(&type_config_zoom, status_zoom.cur_pos);
            busy_wait_after_phase();
        }else if('9' == inByte){
            status_zoom.cur_pos += 3;
            status_zoom.cur_pos %= 4;
            stepper_do_phase(&type_config_zoom, status_zoom.cur_pos);
            busy_wait_after_phase();
        }
        // if any
        {
            p[1] = digitalRead(14) + '0';
            p[2] = digitalRead(15) + '0';
            p[3] = '\n';
            len = 3;
            vcp_write(p[0]);
            vcp_write(p[1]);
            vcp_write(p[2]);
        }
#else
        // process write "status_zoom.pos_should_be" and "status_focus.pos_should_be"
        push_char_pop_package(inByte, p, &len, _MAX_PACKAGE_LEN_);
        if (len > 0){
            process_package_nack_ack_include(p, len);
        }
#endif
    }else{
        // because move steppers use "busy_wait", will block main loop work,
        // so when virtual serial port is not busy, move steppers.
        // of cause should when serial port is not busy either.
        if(E_SELF_TEST_OK == status_zoom.self_tested){
            if(status_zoom.cur_pos < status_zoom.pos_should_be){
                status_zoom.cur_pos ++;
                stepper_do_phase(&type_config_zoom, status_zoom.cur_pos + status_zoom.phase_when_step_0);
            }else if(status_zoom.cur_pos > status_zoom.pos_should_be){
                status_zoom.cur_pos --;
                stepper_do_phase(&type_config_zoom, status_zoom.cur_pos + status_zoom.phase_when_step_0);
            }
        }
		
        // from NOT moving to moving, delay more time.
        if(false == is_moving){
            busy_wait_after_phase();
        }
        busy_wait_after_phase();

							  
        if(E_SELF_TEST_OK == status_zoom.self_tested){
            if(status_zoom.cur_pos < status_zoom.pos_should_be){
                status_zoom.cur_pos ++;
                stepper_do_phase(&type_config_zoom, status_zoom.cur_pos + status_zoom.phase_when_step_0);
            }else if(status_zoom.cur_pos > status_zoom.pos_should_be){
                status_zoom.cur_pos --;
                stepper_do_phase(&type_config_zoom, status_zoom.cur_pos + status_zoom.phase_when_step_0);
            }
        }
		
        if(E_SELF_TEST_OK == status_focus.self_tested){
            if(status_focus.cur_pos < status_focus.pos_should_be){
                status_focus.cur_pos ++;
                stepper_do_phase(&type_config_focus, status_focus.cur_pos + status_focus.phase_when_step_0);
            }else if(status_focus.cur_pos > status_focus.pos_should_be){
                status_focus.cur_pos --;
                stepper_do_phase(&type_config_focus, status_focus.cur_pos + status_focus.phase_when_step_0);
            }
        }
        // from NOT moving to moving, delay more time.
        if(false == is_moving){
            busy_wait_after_phase();
        }
        busy_wait_after_phase();

//        // from moving to NOT moving, emit signal...
//        if((true == is_moving) &&
//           (status_zoom.cur_pos == status_zoom.pos_should_be ) &&
//           (status_focus.cur_pos != status_focus.pos_should_be)
//           ){
//        }

        if((status_zoom.cur_pos  != status_zoom.pos_should_be ) ||
           (status_focus.cur_pos != status_focus.pos_should_be)
           ){
            is_moving = true;
        }else{
		    if(is_moving){
			    // notify move done
#ifdef _NOTIFY_MOVE_DONE_
                p[0] = 0xAC;
                p[1] = ACK_NOTIFY;
                p[2] = NOTOFY_MOVE_DONE;
				vcp_write_payload(p, 3);
#endif
			}
            is_moving = false;
        }
    }

    // button
    if(sp_available()){
        static int sp_len = 0;
        static uint8_t sp_buf[6];
        uint8_t tmp;
        tmp = sp_read();
        if (0 == sp_len)
        {
            if (0xFC == tmp)
            {
                sp_buf[sp_len] = tmp;
                sp_len++;
            }
        }else{
            sp_buf[sp_len] = tmp;
            sp_len++;

            if(6 == sp_len){
                sp_len = 0;
                // recved one package
                if (sp_buf[5] == (
                    sp_buf[0] + 
                    sp_buf[1] + 
                    sp_buf[2] + 
                    sp_buf[3] + 
                    sp_buf[4]
                    ))
                {
                    if (      (0x01 == sp_buf[2]) && (0x00 == sp_buf[3])) {
                        p[0] = 0xAC;
                        p[1] = ACK_NOTIFY;
                        p[2] = NOTIFY_KEY_ZOOM_SUB;
                        vcp_write_payload(p, 3);
                    }else if ((0x01 == sp_buf[2]) && (0x01 == sp_buf[3])) {
                        p[0] = 0xAC;
                        p[1] = ACK_NOTIFY;
                        p[2] = NOTIFY_KEY_ZOOM_ADD;
                        vcp_write_payload(p, 3);
                    }else if ((0x02 == sp_buf[2]) && (0x00 == sp_buf[3])) {
                        p[0] = 0xAC;
                        p[1] = ACK_NOTIFY;
                        p[2] = NOTIFY_KEY_FOCUS_SUB;
                        vcp_write_payload(p, 3);
                    }else if ((0x02 == sp_buf[2]) && (0x01 == sp_buf[3])) {
                        p[0] = 0xAC;
                        p[1] = ACK_NOTIFY;
                        p[2] = NOTIFY_KEY_FOCUS_ADD;
                        vcp_write_payload(p, 3);
                    }else if ((0x03 == sp_buf[2]) && (0x00 == sp_buf[3])) {
                        p[0] = 0xAC;
                        p[1] = ACK_NOTIFY;
                        p[2] = NOTIFY_KEY_AUTO_FOCUS;
                        vcp_write_payload(p, 3);
                    }
                }
            }
        }
    }
}
