#ifndef __OBU_MSG_H__
#define __OBU_MSG_H__

#include <unistd.h>
#include <stdint.h>  

#define DEVICE_ID_SIZE 3 

struct obu_tcp_header_t{
    enum comm_type{
        ETH = 0x0001
    };
    enum target_type{ 
        T_OBU = 3,
        T_CLIENT = 4
    }; 
    enum device_type{
        D_OBU = 0xBD, 
        D_CLIENT = 0xCE
    };
    uint16_t packet_type;
    uint8_t current_sequence;
    uint16_t payload_size;
    uint8_t device_type;
    uint8_t device_id[DEVICE_ID_SIZE];

} __attribute__((packed));

#endif