#ifndef PIBOT_DATA_FRAME_H_
#define PIBOT_DATA_FRAME_H_

enum MESSAGE_ID{
    ID_GET_VERSION = 0,
    ID_SET_ROBOT_PARAMTER = 1,
    ID_GET_ROBOT_PARAMTER = 2,
    ID_INIT_ODOM = 3,
    ID_SET_VELOCITY = 4,
    ID_GET_ODOM = 5,
    ID_GET_PID_DATA = 6,
    ID_GET_IMU_DATA = 7,
    ID_MESSGAE_MAX
};

class Notify{
    public:
        virtual void update(const MESSAGE_ID id, void* data) = 0;
};


class Dataframe{
    public:
        virtual bool init()=0;
        virtual void register_notify(const MESSAGE_ID id, Notify* _nf)=0;
        virtual bool data_recv(unsigned char c)=0;
        virtual bool data_parse()=0;
        virtual bool interact(const MESSAGE_ID id)=0;
};

#endif