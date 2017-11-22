#ifndef LENS_MOTOR_H
#define LENS_MOTOR_H

class Lens_motor
{
public:
  // max and min can be
  Lens_motor(int detect, 
             int aplus, 
             int aminus, 
             int bplus, 
             int bminus,
             unsigned int  max_pos,
             unsigned int  min_pos,
             unsigned int  jump_pos); 
  bool setter(unsigned int pos);
  bool touch();
  unsigned int getter();

  bool find_jump();
  void calibrate();

  bool looper();

  bool moving();

private:
  int DTCT;
  int AP;
  int AM;
  int BP;
  int BM;
  unsigned int  max_tar;
  unsigned int  min_tar;
  unsigned int  cali_jump_pos;

  unsigned int cur;
  unsigned int tar;
  bool         force_notify;

  unsigned char phase;

  void phase_add();
  void phase_sub();
};

#endif
