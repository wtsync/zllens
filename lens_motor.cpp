#include "lens_motor.h"
#include "Arduino.h"

Lens_motor::Lens_motor(int detect, 
                       int aplus, 
                       int aminus, 
                       int bplus, 
                       int bminus,
                       unsigned int  max_pos,
                       unsigned int  min_pos,
                       unsigned int  jump_pos)
{
  DTCT = detect;
  AP = aplus;
  AM = aminus;
  BP = bplus;
  BM = bminus;

  max_tar = max_pos;
  min_tar = min_pos;
  cali_jump_pos = jump_pos;

  cur = 0;
  tar = 0;
  force_notify = false;

  phase = 0;
}

bool Lens_motor::setter(unsigned int pos)
{
  // if((pos > max_tar) || (pos < min_tar))
  // {
  //   return false;
  // }
  if (pos > min_tar)
  {
    pos = min_tar;
  }
  if (pos < min_tar)
  {
    pos = min_tar;
  }
  tar = pos;
  force_notify = true;
  return true;
}
bool Lens_motor::touch(){
  force_notify = true;
  return true;
}
unsigned int Lens_motor::getter()
{
  return tar;
}
bool Lens_motor::find_jump()
{
  // lock...
  // goto 0
  while(HIGH == digitalRead(DTCT))
  {
    phase_sub();
    delay(5);
  }
  // goto 1
  while(LOW == digitalRead(DTCT))
  {
    phase_add();
    delay(5);
  }
  
  // do find
  // goto 0
  while(HIGH == digitalRead(DTCT))
  {
    phase_sub();
    delay(5);
  }
  // goto 1
  while(LOW == digitalRead(DTCT))
  {
    phase_add();
    delay(5);
  }
  return true;
}
void Lens_motor::calibrate()
{
  cur = cali_jump_pos;
  tar = cali_jump_pos;
}
bool Lens_motor::looper()
{
    // move one phase
  if (cur > tar )
  {
    cur --;
    phase_sub();
  }else if(cur < tar)
  {
    cur ++;
    phase_add();
  }
}

bool Lens_motor::moving()
{
    if( cur == tar )
    {
      return true;
    }

  return false;
}

void Lens_motor::phase_add()
{
  phase++;
  if(phase >= 4)
  {
    phase = 0;
  }
  digitalWrite(AP, ((2 == phase ) || (3 == phase) )? HIGH : LOW);
  digitalWrite(AM, ((0 == phase ) || (1 == phase) )? HIGH : LOW);
  digitalWrite(BP, ((1 == phase ) || (2 == phase) )? HIGH : LOW);
  digitalWrite(BM, ((0 == phase ) || (3 == phase) )? HIGH : LOW);
}
void Lens_motor::phase_sub()
{
  if(0 == phase)
  {
    phase = 4;
  }
  phase--;
  digitalWrite(AP, ((2 == phase ) || (3 == phase) ) ? HIGH : LOW);
  digitalWrite(AM, ((0 == phase ) || (1 == phase) ) ? HIGH : LOW);
  digitalWrite(BP, ((1 == phase ) || (2 == phase) ) ? HIGH : LOW);
  digitalWrite(BM, ((0 == phase ) || (3 == phase) ) ? HIGH : LOW);
}

