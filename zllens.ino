#include "lens_motor.h"
#include "package.h"
#include "zllens.h"
#include "calc_afer.h"
#include "cmd_process.h"

// cmd
#ifndef E_CMD_CALC_AF_SEND_LAP
#define E_CMD_CALC_AF_SEND_LAP 2
#endif

#ifndef E_CMD_MOVE
#define E_CMD_MOVE 1
#endif

const char* build_ver = __DATE__ " " __TIME__ ;
const char* err_move_parser = "err, move parser";
const char* err_idle_move = "err, idle move";
const char* err_moving_move = "err, moving move";
const char* err_unkown_state = "err, unknown state";
const char* err_wait_lap_no_lap = "err, wait lap no lap";


class Lens_motor zm(PA5, PA6, PA7, PB0, PA14,1200, 1, 600);
class Lens_motor fm(PA1, PA2, PA3, PA4, PA15,950,  1, 500);
class Calc_afer  calc_afer(1, 1200);

unsigned int last_ms = 0;
enum ZLLENS_FSM fsm = E_IDLE;
void setup()
{
    zm.find_jump();
    zm.calibrate();
    fm.find_jump();
    fm.calibrate();
    Serial.begin();
    last_ms = millis();
    iap_setup();
}
bool last_5ms_notify = false;
bool is_state_sensitive(unsigned char cmd){
    return ((E_MOVE == cmd) || (E_SEND_LAP == cmd));
}
void loop()
{
	unsigned char p[MAX_PAYLOAD_LEN];
	unsigned int  len;
    unsigned char ack[MAX_PAYLOAD_LEN];
    unsigned int  ack_len;
  unsigned int new_z, new_f, z, f;
  bool calc_af_later;
  unsigned char range, gap;
  unsigned int lap;
  bool f_valid;
    unsigned char cmd;

  // payload_loop()
  // payload_available()
  // payload_read(p, &len, MAX_PAYLOAD_LEN)
recv_payload_loop(p, &len, MAX_PAYLOAD_LEN);
  if(len > 1){
      cmd = p[0];

      // IDLE: MOVE
      // MOVING: 
      // WAIT_LAP: LAP
      // any: IAP, GET_VER

      // state sensitive commands
      if(is_state_sensitive(cmd)){
          if(E_IDLE == fsm){
              // process move
              if(E_MOVE == cmd){
                  if(parser_move(p, len, zm.getter(), fm.getter(), &new_z, &new_f, &calc_af_later, &range, &gap))
                  {
                      zm.setter(new_z);
                      fm.setter(new_f);
                      if(calc_af_later){
                          // calc parameters
                          calc_afer.reset(new_f, range, gap, &new_f);

                          // start
                          fsm = E_CALC_AF_MOVING;
                          fm.setter(new_f);
                          zm.touch();
                          last_5ms_notify = 0;
                      }
                  }else
                  {
//                    send_payload(err_move_parser, sizeof(err_move_parser));
                  }
              }else
              {
//                send_payload(err_idle_mov, sizeof(err_idle_mov));
              }
          }else if(E_CALC_AF_MOVING == fsm){
//                send_payload(err_moving_move, sizeof(err_moving_move));
              fsm = E_IDLE;
          }else if(E_CALC_AF_WAIT_LAP == fsm){
              if(E_SEND_LAP == cmd){
                  if(parser_send_lap(p, len, &lap)){
                      f_valid = calc_afer.set_lap_get_next_f(lap, &new_f);

                      if(false == f_valid){
                          // finish
//                          send_payload(finish_move, sizeof(finish_move));
                          calc_afer.get_best_f(&new_f, &lap);
                          fm.setter(new_f);
                          fsm = E_IDLE;
                      }else{
                          fsm = E_CALC_AF_MOVING;
                          fm.setter(new_f);
                          zm.touch();
                          last_5ms_notify = 0;
                      }
                  }else{
                      // error
//                      send_payload(err_wait_lap_no_lap, sizeof(err_wait_lap_no_lap));
                      fsm = E_IDLE;
                  }
              }else{
//                send_payload(err_unkown_state, sizeof(err_unkown_state));
                  fsm = E_IDLE;
              }
          }
      }else
      {
        // get ver
        if(E_GET_VER == cmd)
        {
          // MIN(sizeof("Aug 30 2017 18:32:12"), 0x10)
          ack_len = sizeof(build_ver) < MAX_PAYLOAD_LEN ? sizeof(build_ver) : MAX_PAYLOAD_LEN;
          memcpy(ack, build_ver, ack_len);
          send_payload(ack, ack_len);
        }
          // IAP
          if(E_IAP == cmd){
            if(parser_iap_read(p, len, &z)){
                iap_read(z, &f);
            } else if(parser_iap_write(p, len, &z, &f)){
                iap_write(z, f);

                iap_read(z, &f);
            }
              assemble_iap_ack(z, f, p, len, ack, &ack_len);
              send_payload(ack, ack_len);
          }
      }
  }

	// align 5ms, if now is not big enough
    if(millis() < (last_ms + 5)){
        return;
    }
    last_ms = millis();

    if ( (!zm.moving()) && (!fm.moving()) && (E_CALC_AF_MOVING == fsm) )
    {
      // need lap 
      fsm = E_CALC_AF_WAIT_LAP;
      assemble_need_lap_payload(ack, &ack_len);
      send_payload(ack, ack_len);
    }

    zm.looper();
    fm.looper();
}
