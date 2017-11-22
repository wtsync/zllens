#ifndef CMD_PROCESS_H
#define CMD_PROCESS_H

bool parser_move(unsigned char * p, unsigned int len, 
    unsigned int    old_z, 
    unsigned int    old_f, 
    unsigned int  * new_z, 
    unsigned int  * new_f, 
    bool          * calc_af_later, 
    unsigned char * range, 
    unsigned char * gap);

bool parser_send_lap(unsigned char* p, unsigned int len, unsigned int * lap);
bool parser_iap_read(unsigned char* p, unsigned int len,
					 unsigned int * z);
bool parser_iap_write(unsigned char* p, unsigned int len,
	   				  unsigned int * z, unsigned int * f);
// bool parser_get_build_ver(unsigned char* p, unsigned int len);

bool assemble_need_lap_payload(unsigned char* p, unsigned int* len);
bool assemble_iap_ack(unsigned int z, unsigned int f, 
					  unsigned char* p, unsigned int len,
                      unsigned char* ack, unsigned int * ack_len);

void iap_setup();
bool iap_read(unsigned int z, unsigned int * f);
bool iap_write(unsigned int z, unsigned int f);

#endif

