#ifndef ZLLENS_H
#define ZLLENS_H

enum ZLLENS_FSM
{
    E_IDLE,
    E_CALC_AF_MOVING,
    E_CALC_AF_WAIT_LAP
};

enum ZLLENS_CMD
{
    E_MOVE,
    E_SEND_LAP,
    E_IAP,
    E_GET_VER
};

#endif
