#include "neuronbot2_bringup/base_driver.hpp"
// #include "neuronbot2_bringup/data_holder.h"

#include "std_msgs/msg/float32_multi_array.hpp"
// #include "neuronbot2_bringup/serial_transport.h"
// #include "neuronbot2_bringup/simple_dataframe_master.h"
// #include <boost/assign/list_of.hpp>



#if 0
BaseDriver* BaseDriver::instance = NULL;
#endif

const unsigned long BAUDRATE = 115200;

BaseDriver::BaseDriver() 
: Node("neuronbot3_driver",rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "BaseDriver startup");

    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *){});

    std::string serial_port = declare_parameter("serial_port", "/dev/neuronbot2");

    RCLCPP_INFO(get_logger(), "Connecting to serial: '%s'", serial_port.c_str());
    try {
        // create a serial device with immediate timeout
        serial_ = std::make_unique<serial::Serial>(serial_port, BAUDRATE);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), e.what());
        throw;
    }

    run();
    // cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
        // "cmd_vel", rclcpp::QoS(10), [=](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {cmd_vel_cb(msg); });
    // work_loop();
#if 0
    // trans = std::make_shared<Serial_transport>("/dev/neuronbot2", 115200);
    // frame = std::make_shared<Simple_dataframe>(trans.get());
    // trans = boost::make_shared<Serial_transport>("/dev/neuronbot2", 115200);
    // frame = boost::make_shared<Simple_dataframe>(trans.get());

#endif

    // if (trans->init())
    // {
        // RCLCPP_INFO(get_logger(), "connected to main board");
    // } else
    // {
        // RCLCPP_ERROR(get_logger(), "Can't connect to main board");
    // }
    // RCLCPP_INFO(get_logger(), "yoyoyo done here");

    #if 0
    //init config
    bdg.init(&Data_holder::get()->parameter);

    trans = boost::make_shared<Serial_transport>(bdg.port, bdg.buadrate);
    frame = boost::make_shared<Simple_dataframe>(trans.get());


    ROS_INFO("BaseDriver startup");
    if (trans->init())
    {
        ROS_INFO("connected to main board");
    } else
    {
        ROS_ERROR("oops!!! can't connect to main board");
        return;
    }

    ros::Duration(2).sleep(); //wait for device
    ROS_INFO("end sleep");
    
    frame->init();

    frame->interact(ID_GET_VERSION);

    ROS_INFO("robot version:%s build time:%s", Data_holder::get()->firmware_info.version,
                                        Data_holder::get()->firmware_info.time);
    
    init_cmd_odom();

    init_pid_debug();

    read_param();
    #endif
#if 0
    init_imu();
#endif
}
BaseDriver::~BaseDriver()
{
    #if 0
if (instance != NULL)
    delete instance;
    #endif
}

void BaseDriver::run()
{
    cmd_vel_callback();
}
void BaseDriver::work_loop()
{
    RCLCPP_INFO(get_logger(), "Me and my friends in the work loop");
    #if 0
    ros::Rate loop(1000);
    while (rclcpp::ok())
    {
        //boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        update_param();

        update_odom();

        update_pid_debug();

        update_speed();

		#if 0
        if (Data_holder::get()->parameter.imu_type == 'E')
            update_imu();
        #endif
		
        loop.sleep();

	    ros::spinOnce();
    }
    #endif
}

void BaseDriver::cmd_vel_callback()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
        {
                RCLCPP_INFO(this->get_logger(), "hey hey come on cmd_vel %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);

            #ifdef TEST
                Data_holder* dh = Data_holder::get();
                dh->get()->velocity.v_liner_x = msg->linear.x * 100;
                dh->get()->velocity.v_liner_y = msg->linear.x * 100;
                dh->get()->velocity.v_angular_z = msg->angular.z * 100;

                Message msg_sent(4, (unsigned char*)&dh->velocity, sizeof(dh->velocity));
                std::vector<uint8_t> data((unsigned char*)&msg_sent, (unsigned char*)&msg_sent+sizeof(msg_sent.head)+ msg_sent.head.length+1);
                this->serial_->write(data);
            #endif
                RCLCPP_INFO(this->get_logger(), "Message sent");
        }
    );
}
#if 0
void BaseDriver::init_cmd_odom()
{
    frame->interact(ID_INIT_ODOM);

    ROS_INFO_STREAM("subscribe cmd topic on [" << bdg.cmd_vel_topic << "]");
    cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &BaseDriver::cmd_vel_callback, this);

    ROS_INFO_STREAM("advertise odom topic on [" << bdg.odom_topic << "]");
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

    //ROS_INFO_STREAM("subscribe cmd topic on [correct_pos]");
    //correct_pos_sub = nh.subscribe("correct_pos", 1000, &BaseDriver::correct_pos_callback, this);
}

void BaseDriver::init_pid_debug()
{
    if (bdg.out_pid_debug_enable)
    {
        const char* input_topic_name[MAX_MOTOR_COUNT]={"motor1_input", "motor2_input", "motor3_input", "motor4_input"};
        const char* output_topic_name[MAX_MOTOR_COUNT]={"motor1_output", "motor2_output", "motor3_output", "motor4_output"};
        for (size_t i = 0; i < MAX_MOTOR_COUNT; i++)
        {
            
            [i] = nh.advertise<std_msgs::Int32>(input_topic_name[i], 1000);
            pid_debug_pub_output[i] = nh.advertise<std_msgs::Int32>(output_topic_name[i], 1000);
        }
    }
}

#if 0
void BaseDriver::init_imu()
{
    raw_imu_pub = nh.advertise<neuronbot2_msgs::RawImu>("raw_imu", 50);
    raw_imu_msgs.header.frame_id = "imu_link";
    raw_imu_msgs.accelerometer = true;
    raw_imu_msgs.gyroscope = true;
    raw_imu_msgs.magnetometer = true;
}
#endif

void BaseDriver::read_param()
{
    Robot_parameter* param = &Data_holder::get()->parameter;
    memset(param,0, sizeof(Robot_parameter));

    frame->interact(ID_GET_ROBOT_PARAMTER);

    ROS_INFO("RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
        param->wheel_diameter, param->wheel_track,  param->encoder_resolution, 
        param->do_pid_interval, param->kp, param->ki, param->kd, param->ko, 
        param->cmd_last_time, param->max_v_liner_x, param->max_v_liner_y, param->max_v_angular_z,
        param->imu_type);

    bdg.SetRobotParameters();
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
    if (need_update_speed)
    {
        ROS_INFO_STREAM("update_speed");
        need_update_speed = !(frame->interact(ID_SET_VELOCITY));
    }
}
#if 0
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
#endif
#if 0
void BaseDriver::update_imu()
{
    frame->interact(ID_GET_IMU_DATA);
    raw_imu_msgs.header.stamp = ros::Time::now();
    raw_imu_msgs.raw_linear_acceleration.x = Data_holder::get()->imu_data[0];
    raw_imu_msgs.raw_linear_acceleration.y = Data_holder::get()->imu_data[1];
    raw_imu_msgs.raw_linear_acceleration.z= Data_holder::get()->imu_data[2];
    raw_imu_msgs.raw_angular_velocity.x = Data_holder::get()->imu_data[3];
    raw_imu_msgs.raw_angular_velocity.y = Data_holder::get()->imu_data[4];
    raw_imu_msgs.raw_angular_velocity.z = Data_holder::get()->imu_data[5];
    raw_imu_msgs.raw_magnetic_field.x = Data_holder::get()->imu_data[6];
    raw_imu_msgs.raw_magnetic_field.y = Data_holder::get()->imu_data[7];
    raw_imu_msgs.raw_magnetic_field.z = Data_holder::get()->imu_data[8];

    // ROS_INFO("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", raw_imu_msgs.raw_linear_acceleration.x, raw_imu_msgs.raw_linear_acceleration.y, raw_imu_msgs.raw_linear_acceleration.z,
    // raw_imu_msgs.raw_angular_velocity.x, raw_imu_msgs.raw_angular_velocity.y ,raw_imu_msgs.raw_angular_velocity.z,
    // raw_imu_msgs.raw_magnetic_field.x, raw_imu_msgs.raw_magnetic_field.y, raw_imu_msgs.raw_magnetic_field.z);

    raw_imu_pub.publish(raw_imu_msgs);
}
#endif
#endif