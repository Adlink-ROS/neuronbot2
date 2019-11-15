#ifndef PIBOT_SIMPLE_DATAFRAME_MASTER_H_
#define PIBOT_SIMPLE_DATAFRAME_MASTER_H_

#include "simple_dataframe.h"
#include <string.h>
//#include <boost/thread.hpp>
//#include <boost/thread/lock_factories.hpp>

class Transport;
class Simple_dataframe : public Dataframe{
    public:
        Simple_dataframe(Transport* trans=0);
        ~Simple_dataframe();
        void register_notify(const MESSAGE_ID id, Notify* _nf){
        }

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
		Transport* trans; 

       /* boost::thread* recv_thread;
        boost::condition cond_start_recv;
        boost::condition cond_end_recv;
        boost::mutex _lock;
        bool is_run;*/
};
#endif