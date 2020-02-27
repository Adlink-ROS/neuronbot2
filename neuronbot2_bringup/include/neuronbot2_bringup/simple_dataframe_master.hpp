#ifndef PIBOT_SIMPLE_DATAFRAME_MASTER_HPP_
#define PIBOT_SIMPLE_DATAFRAME_MASTER_HPP_

#include <string.h>
#include <stdio.h>
#include <chrono>
#include "data_holder.hpp"
#include "simple_dataframe.hpp"
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"

namespace neuronbot2
{
class Simple_dataframe : public Dataframe{
    public:
        Simple_dataframe(serial::Serial* _trans);
        ~Simple_dataframe();

        bool init();
        bool data_recv(unsigned char c);
        bool data_parse();
        bool interact(const MESSAGE_ID id);

    private:
        bool recv_proc();
        bool send_message(const MESSAGE_ID id);
        bool send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len);
        bool send_message(Message* msg);

        Message active_rx_msg;
        RECEIVE_STATE recv_state;
        serial::Serial* trans;
        typedef std::vector<uint8_t> Buffer;
};
} // neuronbot2
#endif // PIBOT_SIMPLE_DATAFRAME_MASTER_HPP_