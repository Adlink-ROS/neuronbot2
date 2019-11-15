#ifndef DATA_HOLDER_H_
#define DATA_HOLDER_H_

#include <string.h>

#pragma pack(1)


typedef int int32;
typedef short int16;
typedef unsigned short uint16;

struct Robot_firmware{
    char version[16]; //固件版本
    char time[16];  //构建时间
};

struct Robot_parameter{
    union{
        char buff[64];
        struct{   
            unsigned short wheel_diameter;   //轮子直径  mm
            unsigned short wheel_track;      //差分：轮距， 三全向轮：直径，四全向：前后轮距+左右轮距 mm
            unsigned short encoder_resolution;  //编码器分辨率
            unsigned char do_pid_interval;         //pid间隔 (ms)
            unsigned short kp;
            unsigned short ki;
            unsigned short kd;
            unsigned short ko;                  //pid参数比例
            unsigned short cmd_last_time;       //命令持久时间ms 超过该时间会自动停止运动
            unsigned short max_v_liner_x;
            unsigned short max_v_liner_y;
            unsigned short max_v_angular_z;
            unsigned char imu_type;
        };
    };
};

struct Robot_velocity{
    short v_liner_x; //线速度 前>0 cm/s
    short v_liner_y; //差分轮 为0  cm/s
    short v_angular_z; //角速度 左>0 0.01rad/s  100 means 1 rad/s
};

struct Robot_odom{
    short v_liner_x;      //线速度 前>0 后<0  cm/s
    short v_liner_y;      //差分轮 为0        cm/s
    short v_angular_z;    //角速度 左>0 右<0  0.01rad/s   100 means 1 rad/s
    int32 x;              //里程计坐标x       cm (这里long为4字节，下同)
    int32 y;              //里程计坐标y       cm
    short yaw;            //里程计航角        0.01rad     100 means 1 rad
};

struct Robot_pid_data{
    int32 input[4];  //各轮子驱动输入值
    int32 output[4]; //个轮子输出值
};

#pragma pack(0)

class Data_holder{
    public:
        static Data_holder* get(){
            static Data_holder dh;
            return &dh;
        }

        void load_parameter();

        void save_parameter();
    
    private:
        Data_holder(){
            memset(&firmware_info, 0, sizeof(struct Robot_firmware));
            memset(&parameter, 0, sizeof(struct Robot_parameter));
            memset(&velocity, 0, sizeof(struct Robot_velocity));
            memset(&odom, 0, sizeof(struct Robot_odom));
            memset(&pid_data, 0, sizeof(struct Robot_pid_data));
            memset(&imu_data, 0, sizeof(imu_data));
            }
    public:
        struct Robot_firmware  firmware_info;
        struct Robot_parameter  parameter;
        struct Robot_velocity  velocity;
        struct Robot_odom      odom;
        struct Robot_pid_data  pid_data;

        float imu_data[9];
};
#endif