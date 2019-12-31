#ifndef DATA_HOLDER_HPP_
#define DATA_HOLDER_HPP_

#include <string.h>

namespace neuronbot2
{
#pragma pack(1)

typedef int int32;
typedef short int16;
typedef unsigned short uint16;

struct Robot_firmware{
    char version[16]; // Firmware version
    char time[16];  // Building time
};

struct Robot_parameter{
    union{
        char buff[64];
        struct{   
            unsigned short wheel_diameter;      // Diameter of wheel (mm)
            unsigned short wheel_track;         // Differencial wheel system: distance between wheels. (mm)
                                                // 3 omni wheel system: diameter. (mm)
                                                // 4 omni wheel system: distance of front/back wheels + distance of left/right wheels. (mm)
            unsigned short encoder_resolution;  // Resolution of the encoder
            unsigned char do_pid_interval;      // pid interval (ms)
            unsigned short kp;                  // Proportional gain
            unsigned short ki;                  // Integral gain
            unsigned short kd;                  // differential gain
            unsigned short ko;                  // Open loop gain
            unsigned short cmd_last_time;       // Sustained time of a command before stopping. (ms)
            unsigned short max_v_liner_x;       // Maximun linear speed in x direction. (cm/s)
            unsigned short max_v_liner_y;       // Maximun linear speed in y direction. (cm/s) 0 if differential.
            unsigned short max_v_angular_z;     // Maximun angular speed in z direction. (0.01 rad/s)
            unsigned char imu_type;             // Type of imu
        } params;
    };
};

struct Robot_velocity{
    short v_liner_x;    // Linear speed in x direction. >0: forward | <0: backward (cm/s)
    short v_liner_y;    // Linear speed in y direction. 0 if differential.(cm/s) 
    short v_angular_z;  // Angular speed in z direction. >0: turn left | <0: turn right (0.01 rad/s)
};

struct Robot_odom{
    short v_liner_x;    // Linear speed in x direction. >0: forward | <0: backward (cm/s)
    short v_liner_y;    // Linear speed in y direction. 0 if differential.(cm/s) 
    short v_angular_z;  // Angular speed in z direction. >0: turn left | <0: turn right (0.01 rad/s)
    int32 x;            // Odometry x (cm) (4 bytes)
    int32 y;            // Odometry y (cm) (4 bytes)
    short yaw;          // Odometry theta (0.01 rad)
};

struct Robot_pid_data{
    int32 input[4];  // Input of each wheel
    int32 output[4]; // Output of each wheel
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
} // neuronbot2
#endif // DATA_HOLDER_HPP_