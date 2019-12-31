#ifndef PIBOT_SIMPLE_DATAFRAME_MASTER_HPP_
#define PIBOT_SIMPLE_DATAFRAME_MASTER_HPP_

#include <string.h>
#include <stdio.h>
#include <chrono>
#include "data_holder.hpp"
#include "simple_dataframe.hpp"
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"

#define ROS2
#ifdef ROS2


#else

#include "transport.h"
#include <boost/thread.hpp>
#include <boost/thread/lock_factories.hpp>

#endif

// class Transport;
class Simple_dataframe : public Dataframe{
    public:
#ifdef ROS2
        // Simple_dataframe(std::shared_ptr<serial::Serial> _trans);
        Simple_dataframe(serial::Serial* _trans);
#else
        Simple_dataframe(Transport* trans=0);
#endif
        ~Simple_dataframe();
        // void register_notify(const MESSAGE_ID id, Notify* _nf){}

        bool data_recv(unsigned char c);
        bool data_parse();
        bool init();
        bool interact(const MESSAGE_ID id);
    private:
        bool recv_proc();
    private:
        bool send_message(const MESSAGE_ID id);
        bool send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len);
        bool send_message(Message* msg);
    private:
        Message active_rx_msg;

        RECEIVE_STATE recv_state;
#ifdef ROS2
        // std::shared_ptr<serial::Serial> trans;
        serial::Serial* trans;
        typedef std::vector<uint8_t> Buffer;
#else
		Transport* trans; 
#endif

       /* boost::thread* recv_thread;
        boost::condition cond_start_recv;
        boost::condition cond_end_recv;
        boost::mutex _lock;
        bool is_run;*/
};
#endif