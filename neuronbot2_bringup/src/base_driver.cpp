#include "base_driver.h"
#include "data_holder.h"

#include <std_msgs/Float32MultiArray.h>
#include "serial_transport.h"
#include "simple_dataframe_master.h"
#include <boost/assign/list_of.hpp>

BaseDriver* BaseDriver::instance = NULL;

BaseDriver::BaseDriver() : pn("~"), bdg(pn)
{
    //init config
    bdg.init(&Data_holder::get()->parameter);

    trans = boost::make_shared<Serial_transport>(bdg.port, bdg.baudrate);
    frame = boost::make_shared<Simple_dataframe>(trans.get());

    ROS_INFO("[NeuronBot2] BaseDriver startup");
    if (trans->init())
    {
        ROS_INFO("[NeuronBot2] Connected to main board");
    } else
    {
        ROS_ERROR("[NeuronBot2] Oops!!! can't connect to main board");
        return;
    }

    ros::Duration(2).sleep(); //wait for device

    frame->init();
    init_param();
    read_param();

    init_cmd_odom();
    init_pid_debug();
    init_imu();
}

BaseDriver::~BaseDriver()
{
if (instance != NULL)
    delete instance;
}

void BaseDriver::init_cmd_odom()
{
    frame->interact(ID_INIT_ODOM);

    ROS_INFO_STREAM("[NeuronBot2] Subscribe cmd topic on [" << bdg.cmd_vel_topic << "]");
    cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &BaseDriver::cmd_vel_callback, this);

    ROS_INFO_STREAM("[NeuronBot2] Advertise odom topic on [" << bdg.odom_topic << "]");
    odom_pub = nh.advertise<nav_msgs::Odometry>(bdg.odom_topic, 50);

    //init odom_trans
    odom_trans.header.frame_id = bdg.odom_frame;  
    odom_trans.child_frame_id = bdg.base_frame;  

    odom_trans.transform.translation.z = 0;  

    //init odom
    odom.header.frame_id = bdg.odom_frame;  
    odom.pose.pose.position.z = 0.0;
    odom.child_frame_id = bdg.base_frame;  
    odom.twist.twist.linear.y = 0;  

    if (!bdg.publish_tf){
        odom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ;
    
        odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ; 
    }

    need_update_speed = false;
    last_cmd_vel_time = 0.0;
}

void BaseDriver::init_pid_debug()
{
    if (bdg.out_pid_debug_enable)
    {
        const char* input_topic_name[MAX_MOTOR_COUNT]={"motor1_input", "motor2_input", "motor3_input", "motor4_input"};
        const char* output_topic_name[MAX_MOTOR_COUNT]={"motor1_output", "motor2_output", "motor3_output", "motor4_output"};
        for (size_t i = 0; i < MAX_MOTOR_COUNT; i++)
        {
            pid_debug_pub_input[i] = nh.advertise<std_msgs::Int32>(input_topic_name[i], 1000);
            pid_debug_pub_output[i] = nh.advertise<std_msgs::Int32>(output_topic_name[i], 1000);
        }
    }
}

void BaseDriver::init_imu()
{
    ROS_INFO_STREAM("[NeuronBot2] Advertise imu topic on [raw_imu] and [raw_mag]");
#if 0
    raw_imu_pub = nh.advertise<neuronbot2_msgs::RawImu>("raw_imu", 50);
    raw_imu_msgs.header.frame_id = "imu_link";
    raw_imu_msgs.accelerometer = true;
    raw_imu_msgs.gyroscope = true;
    raw_imu_msgs.magnetometer = true;
#else
    raw_imu_pub = nh.advertise<sensor_msgs::Imu>("raw_imu", 50);
    raw_imu_msgs.header.seq = 0;    
    raw_imu_msgs.header.stamp = ros::Time::now();
    raw_imu_msgs.header.frame_id = "imu_link";
    raw_imu_msgs.orientation.x = 0;
    raw_imu_msgs.orientation.y = 0;
    raw_imu_msgs.orientation.z = 0;
    raw_imu_msgs.orientation.w = 0;

    raw_imu_msgs.angular_velocity.x = 0;
    raw_imu_msgs.angular_velocity.y = 0;
    raw_imu_msgs.angular_velocity.z = 0;

    raw_imu_msgs.linear_acceleration.x = 0;
    raw_imu_msgs.linear_acceleration.y = 0;
    raw_imu_msgs.linear_acceleration.z = 0;

    raw_imu_msgs.orientation_covariance.fill(0.0);
    raw_imu_msgs.orientation_covariance[0] = -1; // we don't have estimation for orientation
    raw_imu_msgs.angular_velocity_covariance.fill(0.0);
    //raw_imu_msgs.angular_velocity_covariance[0] = 0.01;
    //raw_imu_msgs.angular_velocity_covariance[4] = 0.01;
    //raw_imu_msgs.angular_velocity_covariance[8] = 0.01;
    raw_imu_msgs.linear_acceleration_covariance.fill(0.0);
    //raw_imu_msgs.linear_acceleration_covariance[0] = 0.01;
    //raw_imu_msgs.linear_acceleration_covariance[4] = 0.01;
    //raw_imu_msgs.linear_acceleration_covariance[8] = 0.01;
    
    raw_mag_pub = nh.advertise<sensor_msgs::MagneticField>("raw_mag", 50);
    raw_mag_msgs.header.seq = 0;    
    raw_mag_msgs.header.stamp = raw_imu_msgs.header.stamp;
    raw_mag_msgs.magnetic_field.x = 0;
    raw_mag_msgs.magnetic_field.y = 0;
    raw_mag_msgs.magnetic_field.z = 0;
    raw_mag_msgs.magnetic_field_covariance.fill(0.0);
#endif    
}

void BaseDriver::init_param()
{
    frame->interact(ID_GET_VERSION);
    std::string build_version(Data_holder::get()->firmware_info.version);
    std::string build_time(Data_holder::get()->firmware_info.time);    
    ROS_INFO("[NeuronBot2] Motor controller version: %s build time: %s", build_version.c_str(), build_time.c_str());

#ifdef WRITE_ROBOT_PARAM
    // replace all '.' to '_' for namespace comparison
    std::replace(build_version.begin(), build_version.end(), '.', '_');
    
    Robot_parameter* param_ptr = &Data_holder::get()->parameter;
    //memset(param_ptr, 0, sizeof(Robot_parameter));
    std::string key;
    int value;

    key = build_version + "/" + "wheel_diameter";
    pn.param<int>(key, value, 84);
    param_ptr->wheel_diameter = (unsigned short)value;
    
    key = build_version + "/" + "wheel_track";
    pn.param<int>(key, value, 225);
    param_ptr->wheel_track = (unsigned short)value;

    key = build_version + "/" + "encoder_resolution";
    pn.param<int>(key, value, 1428);
    param_ptr->wheel_track = (unsigned short)value;

    key = build_version + "/" + "do_pid_interval";
    pn.param<int>(key, value, 10);
    param_ptr->do_pid_interval = (unsigned char)value;

    key = build_version + "/" + "kp";
    pn.param<int>(key, value, 75);
    param_ptr->kp = (unsigned short)value;
        
    key = build_version + "/" + "ki";
    pn.param<int>(key, value, 2500);
    param_ptr->ki = (unsigned short)value;

    key = build_version + "/" + "kd";
    pn.param<int>(key, value, 0);
    param_ptr->kd = (unsigned short)value;

    key = build_version + "/" + "ko";
    pn.param<int>(key, value, 10);
    param_ptr->ko = (unsigned short)value;        

    key = build_version + "/" + "cmd_last_time";
    pn.param<int>(key, value, 10);
    param_ptr->cmd_last_time = (unsigned short)value;   

    key = build_version + "/" + "max_v_liner_x";
    pn.param<int>(key, value, 40);
    param_ptr->max_v_liner_x = (unsigned short)value;   

    key = build_version + "/" + "max_v_liner_y";
    pn.param<int>(key, value, 0);
    param_ptr->max_v_liner_y = (unsigned short)value;   

    key = build_version + "/" + "max_v_angular_z";
    pn.param<int>(key, value, 150);
    param_ptr->max_v_angular_z = (unsigned short)value;    

    key = build_version + "/" + "imu_type";
    pn.param<int>(key, value, 69);
    param_ptr->imu_type = (unsigned char)value;        
    
    bool ret = frame->interact(ID_SET_ROBOT_PARAMTER);     
    if (!ret) 
    {
        ROS_ERROR("[NeuronBot2] Failed to set RobotParameters!");
    }
    else 
    {
        ROS_INFO("[NeuronBot2] Set RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
            param_ptr->wheel_diameter, param_ptr->wheel_track,  param_ptr->encoder_resolution, 
            param_ptr->do_pid_interval, param_ptr->kp, param_ptr->ki, param_ptr->kd, param_ptr->ko, 
            param_ptr->cmd_last_time, param_ptr->max_v_liner_x, param_ptr->max_v_liner_y, param_ptr->max_v_angular_z,
            param_ptr->imu_type);    
    }

    ros::Rate loop(5);
    loop.sleep();
#endif    
}

void BaseDriver::read_param()
{
    Robot_parameter* param_ptr = &Data_holder::get()->parameter;
    memset(param_ptr, 0, sizeof(Robot_parameter));

    bool ret = frame->interact(ID_GET_ROBOT_PARAMTER);
    if (!ret) 
    {
        ROS_ERROR("[NeuronBot2] Failed to get RobotParameters!");
    }
    else
    {
        ROS_INFO("[NeuronBot2] RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
            param_ptr->wheel_diameter, param_ptr->wheel_track,  param_ptr->encoder_resolution, 
            param_ptr->do_pid_interval, param_ptr->kp, param_ptr->ki, param_ptr->kd, param_ptr->ko, 
            param_ptr->cmd_last_time, param_ptr->max_v_liner_x, param_ptr->max_v_liner_y, param_ptr->max_v_angular_z,
            param_ptr->imu_type);
    }
    bdg.SetRobotParameters();      
}

void BaseDriver::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
    // ROS_INFO_STREAM("cmd_vel:[" << vel_cmd.linear.x << " " << vel_cmd.linear.y << " " << vel_cmd.angular.z << "]");

    Data_holder::get()->velocity.v_liner_x = vel_cmd.linear.x*100;
    Data_holder::get()->velocity.v_liner_y = vel_cmd.linear.y*100;
    Data_holder::get()->velocity.v_angular_z = vel_cmd.angular.z*100;

    need_update_speed = true;
    last_cmd_vel_time = ros::Time::now().toSec();
}

void BaseDriver::work_loop()
{
    ros::Rate loop(1000);
    while (ros::ok())
    {
        update_param();

        update_odom();

        update_pid_debug();

        update_speed();
		
        if (Data_holder::get()->parameter.imu_type == 'E')
            update_imu();
		
        loop.sleep();

	    ros::spinOnce();
    }
}

void BaseDriver::update_param()
{
#ifdef USE_DYNAMIC_RECONFIG
    if (bdg.get_param_update_flag())
    {
        frame->interact(ID_SET_ROBOT_PARAMTER);
        ros::Rate loop(5);
        loop.sleep();
    }
#endif
}

void BaseDriver::update_odom()
{
    frame->interact(ID_GET_ODOM);
    
    ros::Time current_time = ros::Time::now(); 

    float x = Data_holder::get()->odom.x*0.01;
    float y = Data_holder::get()->odom.y*0.01;
    float th = Data_holder::get()->odom.yaw*0.01;

    float vxy = Data_holder::get()->odom.v_liner_x*0.01;
    float vth = Data_holder::get()->odom.v_angular_z*0.01;

    // ROS_INFO("odom: x=%.2f y=%.2f th=%.2f vxy=%.2f vth=%.2f", x, y ,th, vxy,vth);
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //publish_tf
    if (bdg.publish_tf){
        odom_trans.header.stamp = current_time;  
        odom_trans.transform.translation.x = x;  
        odom_trans.transform.translation.y = y;  
        odom_trans.transform.rotation = odom_quat; 
        odom_broadcaster.sendTransform(odom_trans);  
    }

    //publish the message  
    odom.header.stamp = current_time;  
    odom.pose.pose.position.x = x;  
    odom.pose.pose.position.y = y;  
    odom.pose.pose.orientation = odom_quat;    
    odom.twist.twist.linear.x = vxy;  
    odom.twist.twist.angular.z = vth;  
    
    odom_pub.publish(odom);
}

void BaseDriver::update_speed()
{
    static bool cmd_vel_timer = false;
    
    // check cmd_vel_idle_time
    double current_time = ros::Time::now().toSec();
    if (cmd_vel_timer && ((current_time - last_cmd_vel_time) >= bdg.cmd_vel_idle_time))
    {
        // timeout, set speed to zero
        Data_holder::get()->velocity.v_liner_x = 0;
        Data_holder::get()->velocity.v_liner_y = 0;
        Data_holder::get()->velocity.v_angular_z = 0;        

        ROS_DEBUG("No more topic '/%s' received in %.1lf sec, robot stopped", bdg.cmd_vel_topic.c_str(), bdg.cmd_vel_idle_time);
        frame->interact(ID_SET_VELOCITY);
        last_cmd_vel_time = current_time;
        cmd_vel_timer = false;
        need_update_speed = false;
        return;
    }

    if (need_update_speed)
    {    
        frame->interact(ID_SET_VELOCITY);
        cmd_vel_timer = true; 
    }
}

void BaseDriver::update_pid_debug()
{
    if (bdg.out_pid_debug_enable)
    {
        frame->interact(ID_GET_PID_DATA);
        
        for (size_t i = 0; i < MAX_MOTOR_COUNT; i++)
        {
            pid_debug_msg_input[i].data = Data_holder::get()->pid_data.input[i];
            pid_debug_msg_output[i].data = Data_holder::get()->pid_data.output[i];

            pid_debug_pub_input[i].publish(pid_debug_msg_input[i]);
            pid_debug_pub_output[i].publish(pid_debug_msg_output[i]);
        }
    }
}

void BaseDriver::update_imu()
{
    frame->interact(ID_GET_IMU_DATA);
    raw_imu_msgs.header.stamp = ros::Time::now();
#if 0    
    raw_imu_msgs.raw_angular_velocity.y = Data_holder::get()->imu_data[4];
    raw_imu_msgs.raw_angular_velocity.z = Data_holder::get()->imu_data[5];
    raw_imu_msgs.raw_magnetic_field.x = Data_holder::get()->imu_data[6];
    raw_imu_msgs.raw_magnetic_field.y = Data_holder::get()->imu_data[7];
    raw_imu_msgs.raw_magnetic_field.z = Data_holder::get()->imu_data[8];

    // ROS_INFO("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", raw_imu_msgs.raw_linear_acceleration.x, raw_imu_msgs.raw_linear_acceleration.y, raw_imu_msgs.raw_linear_acceleration.z,
    raw_imu_msgs.raw_angular_velocity.x, raw_imu_msgs.raw_angular_velocity.y ,raw_imu_msgs.raw_angular_velocity.z,
    raw_imu_msgs.raw_magnetic_field.x, raw_imu_msgs.raw_magnetic_field.y, raw_imu_msgs.raw_magnetic_field.z);
#else
    raw_imu_msgs.linear_acceleration.x = Data_holder::get()->imu_data[0];
    raw_imu_msgs.linear_acceleration.y = Data_holder::get()->imu_data[1];
    raw_imu_msgs.linear_acceleration.z = Data_holder::get()->imu_data[2];
    raw_imu_msgs.angular_velocity.x = Data_holder::get()->imu_data[3];
    raw_imu_msgs.angular_velocity.y = Data_holder::get()->imu_data[4];
    raw_imu_msgs.angular_velocity.z = Data_holder::get()->imu_data[5];

    raw_mag_msgs.header.stamp = raw_imu_msgs.header.stamp;
    raw_mag_msgs.magnetic_field.x = Data_holder::get()->imu_data[6];
    raw_mag_msgs.magnetic_field.y = Data_holder::get()->imu_data[7];
    raw_mag_msgs.magnetic_field.z = Data_holder::get()->imu_data[8];
#endif
    raw_imu_pub.publish(raw_imu_msgs);
    raw_mag_pub.publish(raw_mag_msgs);
}
