#ifndef PIBOT_SIMPLE_DATAFRAME_HPP_
#define PIBOT_SIMPLE_DATAFRAME_HPP_

#include <string.h>
#include "dataframe.hpp"

namespace neuronbot2
{
static const unsigned short MESSAGE_BUFFER_SIZE = 255;

#define FIX_HEAD 0x5A

struct Head{
    unsigned char flag;     // Head prefix: 0X5A
    unsigned char msg_id;   // Message ID: determine the function and format of the message
    unsigned char length;   // Length of the message
};


struct Message{
    struct Head head;
    unsigned char data[MESSAGE_BUFFER_SIZE];
    unsigned char check;
    unsigned char recv_count;   // Received bytes

    Message(){}
    Message(unsigned char msg_id, unsigned char* data=0,unsigned char len=0){
        head.flag = FIX_HEAD;
        head.msg_id = msg_id;
        head.length = recv_count = len;
        check = 0;

        if (data != 0 && len !=0)
            memcpy(this->data, data, len);
        
        unsigned char* _send_buffer = (unsigned char*)this;

        unsigned int i = 0;
        for (i = 0; i < sizeof(head)+head.length; i++)
            check += _send_buffer[i];
        
        _send_buffer[sizeof(head)+head.length] = check;
    }
};

enum RECEIVE_STATE{
    STATE_RECV_FIX=0,
    STATE_RECV_ID,
    STATE_RECV_LEN,
    STATE_RECV_DATA,
    STATE_RECV_CHECK
};
} // neuronbot2
#endif // PIBOT_SIMPLE_DATAFRAME_HPP_