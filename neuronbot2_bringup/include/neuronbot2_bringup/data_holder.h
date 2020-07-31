#ifndef DATA_HOLDER_H_
#define DATA_HOLDER_H_

#include <string.h>

#pragma pack(1)


typedef int int32;
typedef short int16;
typedef unsigned short uint16;

struct Robot_firmware{
    char version[16];
    char time[16];
};

struct Robot_parameter{
    union{
        char buff[64];
        struct{   
            unsigned short wheel_diameter;
            unsigned short wheel_track;
            unsigned short encoder_resolution;
            unsigned char do_pid_interval;
            unsigned short kp;
            unsigned short ki;
            unsigned short kd;
            unsigned short ko;
            unsigned short cmd_last_time;
            unsigned short max_v_liner_x;
            unsigned short max_v_liner_y;
            unsigned short max_v_angular_z;
            unsigned char imu_type;
        };
    };
};

struct Robot_velocity{
    short v_liner_x;
    short v_liner_y;
    short v_angular_z; 
};

struct Robot_odom{
    short v_liner_x;      
    short v_liner_y;      
    short v_angular_z;    
    int32 x;              
    int32 y;              
    short yaw;            
};

struct Robot_pid_data{
    int32 input[4];  
    int32 output[4]; 
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