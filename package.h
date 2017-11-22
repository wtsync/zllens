#include "Arduino.h"

#define MAX_PAYLOAD_LEN 0x10

void recv_payload_loop(unsigned char* p, unsigned int* len, unsigned int max_len);
void send_payload(unsigned char* p, unsigned int len);
