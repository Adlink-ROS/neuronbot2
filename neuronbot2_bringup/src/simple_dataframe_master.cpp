#include "neuronbot2_bringup/simple_dataframe_master.hpp"
#ifdef ROS2
// Simple_dataframe::Simple_dataframe(std::shared_ptr<serial::Serial> _trans) : trans(_trans)
Simple_dataframe::Simple_dataframe(serial::Serial* _trans) : trans(_trans)
{
    recv_state = STATE_RECV_FIX;
}
#else
Simple_dataframe::Simple_dataframe(Transport* _trans): trans(_trans){
    recv_state = STATE_RECV_FIX;
}
#endif

Simple_dataframe::~Simple_dataframe(){
}

bool Simple_dataframe::init(){
#ifdef ROS2
    // uint32_t a = 5000;
    // trans->setTimeout(a,a,a,a,a);
    // printf("in dataframe init \n");
#else
    trans->set_timeout(5000);
#endif
    return true;
}

bool Simple_dataframe::data_recv(unsigned char c){
    // printf("%02x ", c);
    switch (recv_state){
    case STATE_RECV_FIX:
        if (c == FIX_HEAD){
            memset(&active_rx_msg,0, sizeof(active_rx_msg));
            active_rx_msg.head.flag = c;
            active_rx_msg.check += c;

            recv_state = STATE_RECV_ID;
        }
        else
            recv_state = STATE_RECV_FIX;
        break;
    case STATE_RECV_ID:
        if (c < ID_MESSGAE_MAX){
            active_rx_msg.head.msg_id = c;
            active_rx_msg.check += c;
            recv_state = STATE_RECV_LEN;
        }
        else
            recv_state = STATE_RECV_FIX;
        break;
    case STATE_RECV_LEN:
        active_rx_msg.head.length =c;
        active_rx_msg.check += c;
        if (active_rx_msg.head.length==0)
            recv_state = STATE_RECV_CHECK;
        else
            recv_state = STATE_RECV_DATA;
        break;
    case STATE_RECV_DATA:
        active_rx_msg.data[active_rx_msg.recv_count++] = c;
        active_rx_msg.check += c;
        if (active_rx_msg.recv_count >=active_rx_msg.head.length)
            recv_state  = STATE_RECV_CHECK;
        break;
    case STATE_RECV_CHECK:
        recv_state = STATE_RECV_FIX;
        if (active_rx_msg.check == c){
            //printf("\r\n");
            return true;
        }
        break;
    default:
        recv_state = STATE_RECV_FIX;
    }

    return false;
}

bool Simple_dataframe::data_parse(){
    MESSAGE_ID id = (MESSAGE_ID)active_rx_msg.head.msg_id;

    printf("data_parse:id=%d\r\n", id);

    Data_holder* dh = Data_holder::get();
    switch (id){
    case ID_GET_VERSION:
        memcpy(&dh->firmware_info, active_rx_msg.data, sizeof(dh->firmware_info));
        break;
    case ID_SET_ROBOT_PARAMTER:
        break;
    case ID_GET_ROBOT_PARAMTER:
        memcpy(&dh->parameter, active_rx_msg.data, sizeof(dh->parameter));
        break;
    case ID_INIT_ODOM:
        break;
    case ID_SET_VELOCITY:
        break;
    case ID_GET_ODOM:
        memcpy(&dh->odom, active_rx_msg.data, sizeof(dh->odom));
        break;
    case ID_GET_PID_DATA:
        memcpy(&dh->pid_data, active_rx_msg.data, sizeof(dh->pid_data));
        break;
    case ID_GET_IMU_DATA:
        memcpy(&dh->imu_data, active_rx_msg.data, sizeof(dh->imu_data));
        break;
    default:
        break;
    }

    return true;
}

bool Simple_dataframe::send_message(const MESSAGE_ID id){
    Message msg(id);

    send_message(&msg);

    return true;
}

bool Simple_dataframe::send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len){
    Message msg(id, data, len);

    send_message(&msg);

    return true;
}

bool Simple_dataframe::send_message(Message* msg){
    if (trans == 0)
        return true;
    Buffer data((unsigned char*)msg, (unsigned char*)msg+sizeof(msg->head)+msg->head.length+1);
    trans->write(data);

    return true;
}

bool Simple_dataframe::interact(const MESSAGE_ID id){
    //printf("make command:id=%d\r\n", id);

    Data_holder* dh = Data_holder::get();
    switch (id){
    case ID_GET_VERSION:
        send_message(id);
        break;
    case ID_SET_ROBOT_PARAMTER:
        send_message(id, (unsigned char*)&dh->parameter, sizeof(dh->parameter));
        break;
    case ID_GET_ROBOT_PARAMTER:
        send_message(id);
        break;
    case ID_INIT_ODOM:
        send_message(id);
        break;
    case ID_SET_VELOCITY:
        send_message(id, (unsigned char*)&dh->velocity, sizeof(dh->velocity));
        break;
    case ID_GET_ODOM:
        send_message(id);
        break;
    case ID_GET_PID_DATA:
        send_message(id);
        break;
    case ID_GET_IMU_DATA:
        send_message(id);
    default:
        break;
    }

    if (!recv_proc())
        return false;

    return true;
}

bool Simple_dataframe::recv_proc(){
#ifdef ROS2
#else
    trans->set_timeout(150);
#endif

    bool got=false;

    while(true){
#ifdef ROS2
        std::vector<uint8_t> inbuf;
        while (inbuf.empty()) {
            auto n = trans->read(inbuf, 1);
            if (n == 0) {
                // time out
                printf("timeout \n");
                return false;
            }
        }
        // Buffer data = inbuf;
#else
        Buffer data = trans->read();
#endif
        
        for (unsigned i=0;i<inbuf.size();i++){
            if (data_recv(inbuf[i])){
                got = true;
                break;
            }
        }
        if (got)
            break;
    }

    if (!data_parse())
        return false;

    return true;
}