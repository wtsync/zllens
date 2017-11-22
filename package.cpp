#include "package.h"

unsigned char package[MAX_PAYLOAD_LEN + 3];
unsigned int  pack_len = 0;

bool check_package_sum(unsigned char *pack, unsigned int len)
{
    unsigned char sum = 0;
    for(int i=0; i<(len - 1); i++){
        sum += pack[i];
    }

    return sum == pack[len - 1];
}

enum PACK_STAT
{
    WAIT_HEADER,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_SUM
} pack_stat = WAIT_HEADER;
void recv_payload_loop(unsigned char* p, unsigned int* len, unsigned int max_len)
{
    unsigned char tmp;
    while(Serial.available())
    {
        tmp = Serial.read();

        package[pack_len] = tmp;
        pack_len ++;

        if(WAIT_HEADER == pack_stat){
            if(0x11 == package[pack_len - 1]){
                pack_stat = WAIT_LEN;
            }else {
                pack_len = 0;
            }
        }else if(WAIT_LEN == pack_stat){
            if(tmp <= MAX_PAYLOAD_LEN){
                pack_stat = WAIT_PAYLOAD;
            }else{
                pack_len = 0;
            }
        }else if(WAIT_PAYLOAD == pack_stat){
            if(pack_len == (package[1] + 2)){
                pack_stat = WAIT_SUM;
            }else{
                pack_len = 0;
            }
        }else if(WAIT_SUM == pack_stat){
            if(check_package_sum(package, pack_len)){
                memcpy(p, package + 2, package[1]);
                (*len) = package[1];

                pack_len = 0;

                return;
            }else{
                pack_len = 0;
            }
        }else{
            pack_stat = WAIT_HEADER;
            pack_len = 0;
        }
    }
}

void send_payload(unsigned char* p, unsigned int len)
{
  unsigned char sum = 0x11;
  len = len < MAX_PAYLOAD_LEN ? len : MAX_PAYLOAD_LEN ;
  sum += len;
  Serial.write(0x11);
  Serial.write(len);
  for(int i = 0; i < len; i++){
    sum += p[i];
    Serial.write(p[i]);
  }
  Serial.write(sum);  
}


