#include "zllens.h"
#include "cmd_process.h"
#include <EEPROM.h>

struct CMD_MOVE
{
    unsigned char cmd;
    unsigned char ver;
    unsigned char z_type;
    unsigned char zH;
    unsigned char zL;
    unsigned char f_type;
    unsigned char fH;
    unsigned char fL;
    unsigned char af_type;

    unsigned char af_range;
    unsigned char af_gap;
};
// return should enter E_CALC_AF_WAIT_MOVE
bool parser_move(unsigned char * p, unsigned int len, 
    unsigned int    old_z, 
    unsigned int    old_f, 
    unsigned int  * new_z, 
    unsigned int  * new_f, 
    bool          * calc_af_later, 
    unsigned char * range, 
    unsigned char * gap)

{
    struct CMD_MOVE c;
    unsigned int znew, fnew;
    // return still in calc af loop;
    if(sizeof(c) == len){
        c.cmd     = p[0];
        if (E_MOVE != c.cmd)
        {
            return false;
        }
        c.ver     = p[1];
        c.z_type  = p[2];
        c.zH      = p[3];
        c.zL      = p[4];
        c.f_type  = p[5];
        c.fH      = p[6];
        c.fL      = p[7];
        c.af_type = p[8];

        c.af_range= p[9];
        c.af_gap  = p[10];

        if (1 == c.z_type)
        {
            /* code */
        }
        
        if(2 == c.af_type){
            return true;
        }
    }
    return false;
}

struct CMD_CALC_AF_LAP
{
    unsigned char cmd;
    unsigned char ver;

    unsigned char lap3;
    unsigned char lap2;
    unsigned char lap1;
    unsigned char lap0;
};
bool parser_send_lap(unsigned char* p, unsigned int len, unsigned int * lap)
{
    struct CMD_CALC_AF_LAP c;
    if(sizeof(c) == len){
        c.cmd  = p[0];
        c.ver  = p[1];

        c.lap3 = p[2];
        c.lap2 = p[3];
        c.lap1 = p[4];
        c.lap0 = p[5];

        if( (E_SEND_LAP == c.cmd) &&
            (1 == c.ver) ){
                          (*lap)  = c.lap3;
            (*lap) <<= 8; (*lap) += c.lap2;
            (*lap) <<= 8; (*lap) += c.lap1;
            (*lap) <<= 8; (*lap) += c.lap0;

            return true;
        }
    }
    return false;
}

bool assemble_need_lap_payload(unsigned char* p, unsigned int* len){
  p[0] = E_SEND_LAP;
  p[1] = 1;
  (*len) = 2;

  return true;
}

void iap_setup()
{
    EEPROM.PageBase0 = 0x801F000;
    EEPROM.PageBase1 = 0x801F800;
    EEPROM.PageSize  = 0x400;
}
bool iap_read(unsigned int z, unsigned int * f){
    uint16 dat;
    EEPROM.read(z, &dat);
    (*f) = dat;
    return true;
}
bool iap_write(unsigned int z, unsigned int f){
    EEPROM.write(z, f);
    return true;
}

struct CMD_IAP {
    unsigned char cmd;
    unsigned char ver;

    unsigned char zH;
    unsigned char zL;

    unsigned char is_new_f;
    unsigned char fH;
    unsigned char fL;
};

// bool iap_handle(unsigned char* p, unsigned int len,
//                 unsigned char* ack, unsigned int* ack_len)
// {
//     unsigned int z, f;
//     struct CMD_IAP c;
//     if (sizeof(c) == len)
//     {
//         c.cmd = p[0];
//         c.ver = p[1];
//         c.zH = p[2];
//         c.zL = p[3];
//         c.is_new_f = p[4];
//         c.fH = p[5];
//         c.fL = p[6];
//         if (E_IAP == c.cmd)
//         {
//             z = c.zH;
//             z <<= 8; 
//             z += c.zL;
//             if (c.is_new_f)
//             {
//                 f = c.fH;
//                 f <<= 8;
//                 f += c.fL;
//                 iap_write(z, f);
//             }
//             iap_read(z, &f);
//             ack[0] = p[0];
//             ack[1] = p[1];
//             ack[2] = p[2];
//             ack[3] = p[3];
//             ack[4] = 1;
//             ack[5] = (f >> 8) & 0xFF;
//             ack[6] =  f       & 0xFF;
//             (*ack_len) = sizeof(c);
//             return true;
//         }
//     }

//     return false;
// }

bool parser_iap_read(unsigned char* p, unsigned int len,
                     unsigned int * z)
{
    struct CMD_IAP c;
    if (sizeof(c) == len)
    {
        c.cmd = p[0];
        c.ver = p[1];
        c.zH = p[2];
        c.zL = p[3];
        c.is_new_f = p[4];
        c.fH = p[5];
        c.fL = p[6];
        if (E_IAP != c.cmd)
        {
            (*z) = c.zH;
            (*z) <<= 8; 
            (*z) += c.zL;
           return true;
        }
    }

    return false;
}
bool parser_iap_write(unsigned char* p, unsigned int len,
                      unsigned int * z, unsigned int * f)
{
    struct CMD_IAP c;
    if (sizeof(c) == len)
    {
        c.cmd = p[0];
        c.ver = p[1];
        c.zH = p[2];
        c.zL = p[3];
        c.is_new_f = p[4];
        c.fH = p[5];
        c.fL = p[6];
        if (E_IAP == c.cmd)
        {
            (*z) = c.zH;
            (*z) <<= 8; 
            (*z) += c.zL;

            (*f) = c.fH;
            (*f) <<= 8; 
            (*f) += c.fL;

           return true;
        }
    }

    return false;
}
bool assemble_iap_ack(unsigned int z, unsigned int f, 
                      unsigned char* p, unsigned int len,
                      unsigned char* ack, unsigned int * ack_len)
{
    if (sizeof(struct CMD_IAP) != len)
    {
        ack[0] = p[0];
        ack[1] = p[1];
        ack[2] = (z >> 8) & 0xFF;
        ack[3] =  z       & 0xFF;
        ack[4] = 1;
        ack[5] = (f >> 8) & 0xFF;
        ack[6] =  f       & 0xFF;
        (*ack_len) = len;  // sizeof(struct CMD_IAP)
        return true;
    }

    return false;
}
