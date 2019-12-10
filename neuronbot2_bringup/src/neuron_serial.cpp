#include <chrono>
#include <iomanip>
#include <iostream>
#include "neuronbot2_bringup/neuron_serial.hpp"

NeuronSerial::NeuronSerial()
: Node("neuron_serial", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "Starting neuronbot serial node");
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *){});

    // Set up port parameters    
    std::string port = declare_parameter("port", "/dev/neuronbot2");
    int32_t baudrate = declare_parameter("baudrate", 115200);
    RCLCPP_INFO(get_logger(), "port:%s buadrate:%d", port.c_str(), baudrate);


    std::string cmd_vel_topic = declare_parameter("cmd_vel_topic", "cmd_vel");
    // Set up timer
    // keepalive_timer = create_wall_timer(keepalive_period, [=]() { keepalive_cb(); });
    read_timer = create_wall_timer(uart_poll_period, [=]() { read_cb(); });
    pub_timer = create_wall_timer(pub_period, [=]() {
        update_imu();
        update_odom();
    });
    // Create Publisher
    // pub odom = create_publisher<>("odom", rclcpp::QoS(10));

    // Create Subscriber

    // Initialize serial.
    RCLCPP_INFO(get_logger(), "Connecting to serial: '%s', with baudrate '%d'", port.c_str(), baudrate);
    try {
        serial_ = std::make_unique<serial::Serial>(port, baudrate);
        // serial_ = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(101));
        RCLCPP_INFO(get_logger(), "Connecting to serial: '%s', with baudrate '%d', timeout: %d", serial_->getPort().c_str(), serial_->getBaudrate(), serial_->getTimeout());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), e.what());
        throw;
    }

    rclcpp::sleep_for(2s);

    frame = std::make_shared<Simple_dataframe>(serial_.get());
    parameter_init();
    rclcpp::sleep_for(2s);
    frame->interact(ID_GET_VERSION);
    rclcpp::sleep_for(2s);
    dh = Data_holder::get();
    std::string version(dh->firmware_info.version);
    std::string time(dh->firmware_info.time);
    RCLCPP_INFO(get_logger(), "robot version:%s || build time:%s", version.c_str(), time.c_str());
    init_cmd_odom();
}
//void on_raw_command()
// void keepalive_cb();
void NeuronSerial::read_cb()
{
    Robot_parameter* param = &Data_holder::get()->parameter;
    memset(param,0, sizeof(Robot_parameter));

    // frame->interact(ID_GET_ROBOT_PARAMTER);

    // RCLCPP_INFO(get_logger(),"RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
    //     param->wheel_diameter, param->wheel_track,  param->encoder_resolution, 
    //     param->do_pid_interval, param->kp, param->ki, param->kd, param->ko, 
    //     param->cmd_last_time, param->max_v_liner_x, param->max_v_liner_y, param->max_v_angular_z,
    //     param->imu_type);
}

void NeuronSerial::parameter_init()
{
    // Set up control parameter
    Robot_parameter* rp = &Data_holder::get()->parameter;
    memset(rp ,0, sizeof(Robot_parameter));

    rp->wheel_diameter = 168;
    rp->wheel_track = 288;
    rp->encoder_resolution = 3892;
    rp->do_pid_interval = 10; 
    rp->kp = 75;
    rp->ki = 2500;
    rp->kd = 0;
    rp->ko = 10;
    rp->cmd_last_time = 250;
    rp->max_v_liner_x = 50;
    rp->max_v_liner_y = 0;
    rp->max_v_angular_z = 120;
    rp->imu_type = 69; // 'E'(69) for enablehttps://bitbucket.org/ROScube/azure_cs_luis/src/master/
    
    RCLCPP_INFO(this->get_logger(),"Request write RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
        rp->wheel_diameter, rp->wheel_track,  rp->encoder_resolution, 
        rp->do_pid_interval, rp->kp, rp->ki, rp->kd, rp->ko, 
        rp->cmd_last_time, rp->max_v_liner_x, rp->max_v_liner_y, rp->max_v_angular_z,
        rp->imu_type);
    
    RCLCPP_INFO(this->get_logger(),"Request write RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
        Data_holder::get()->parameter.wheel_diameter, Data_holder::get()->parameter.wheel_track,  Data_holder::get()->parameter.encoder_resolution, 
        Data_holder::get()->parameter.do_pid_interval, Data_holder::get()->parameter.kp, Data_holder::get()->parameter.ki, Data_holder::get()->parameter.kd, Data_holder::get()->parameter.ko, 
        Data_holder::get()->parameter.cmd_last_time, Data_holder::get()->parameter.max_v_liner_x, Data_holder::get()->parameter.max_v_liner_y,Data_holder::get()->parameter.max_v_angular_z,
        Data_holder::get()->parameter.imu_type);
    frame->interact(ID_SET_ROBOT_PARAMTER);
}

void NeuronSerial::on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "hey hey come on cmd_vel %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);
    Data_holder::get()->velocity.v_liner_x = msg->linear.x * 100;
    Data_holder::get()->velocity.v_liner_y = msg->linear.x * 100;
    Data_holder::get()->velocity.v_angular_z = msg->angular.z * 100;
    bool a = frame->interact(ID_SET_VELOCITY);
    RCLCPP_INFO(this->get_logger(), "Message sent");
}

void NeuronSerial::init_cmd_odom()
{
    frame->interact(ID_INIT_ODOM);
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(1), [=](geometry_msgs::msg::Twist::SharedPtr msg) {on_motor_move(msg); });
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(4));
    raw_imu_pub = create_publisher<sensor_msgs::msg::Imu>("raw_imu", rclcpp::QoS(1));
    raw_mag_pub = create_publisher<sensor_msgs::msg::MagneticField>("raw_mag", rclcpp::QoS(1));
}

void NeuronSerial::update_odom()
{
    frame->interact(ID_GET_ODOM);
    RCLCPP_INFO(get_logger(), "1");
    auto now = get_clock()->now();
    float x = Data_holder::get()->odom.x*0.01;
    float y = Data_holder::get()->odom.y*0.01;
    float th = Data_holder::get()->odom.yaw*0.01;

    float vxy = Data_holder::get()->odom.v_liner_x*0.01;
    float vth = Data_holder::get()->odom.v_angular_z*0.01;

    RCLCPP_INFO(get_logger(), "odom: x=%.2f y=%.2f th=%.2f vxy=%.2f vth=%.2f", x, y ,th, vxy,vth);
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th);



    //publish the message  
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = "odom";
    odom->child_frame_id = "base_link";
    odom->header.stamp = now;
    odom->pose.pose.position.x = x;  
    odom->pose.pose.position.y = y; 
    odom->pose.pose.orientation.x = q.x(); 
    odom->pose.pose.orientation.y = q.y(); 
    odom->pose.pose.orientation.z = q.z(); 
    odom->pose.pose.orientation.w = q.w(); 
    odom->twist.twist.linear.x = vxy;  
    odom->twist.twist.angular.z = vth;  
    odom->twist.covariance.fill(0.0);
    
    RCLCPP_INFO(get_logger(), "pub from %s to %s", odom->header.frame_id.c_str(), odom->child_frame_id.c_str());
    odom_pub->publish(std::move(odom));
    // RCLCPP_INFO(get_logger(), "pub from %s to %s", odom->header.frame_id.c_str(), odom->child_frame_id.c_str());
}

void NeuronSerial::update_imu()
{
    frame->interact(ID_GET_IMU_DATA);

    const rclcpp::Time & now = get_clock()->now();
    auto raw_imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    

    raw_imu_msg->header.frame_id = "imu_link";
    raw_imu_msg->header.stamp = now;    
    raw_imu_msg->linear_acceleration.x = Data_holder::get()->imu_data[0];
    raw_imu_msg->linear_acceleration.y = Data_holder::get()->imu_data[1];
    raw_imu_msg->linear_acceleration.z= Data_holder::get()->imu_data[2];
    raw_imu_msg->angular_velocity.x = Data_holder::get()->imu_data[3];
    raw_imu_msg->angular_velocity.y = Data_holder::get()->imu_data[4];
    raw_imu_msg->angular_velocity.z = Data_holder::get()->imu_data[5];

    auto raw_mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
    raw_mag_msg->header.frame_id = "imu_link";
    raw_mag_msg->header.stamp = now;
    raw_mag_msg->magnetic_field.x = Data_holder::get()->imu_data[6];
    raw_mag_msg->magnetic_field.y = Data_holder::get()->imu_data[7];
    raw_mag_msg->magnetic_field.z = Data_holder::get()->imu_data[8];
    
    RCLCPP_INFO(this->get_logger(), "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", raw_imu_msg->linear_acceleration.x, raw_imu_msg->linear_acceleration.y, raw_imu_msg->linear_acceleration.z,
    raw_imu_msg->angular_velocity.x, raw_imu_msg->angular_velocity.y ,raw_imu_msg->angular_velocity.z,
    raw_mag_msg->magnetic_field.x, raw_mag_msg->magnetic_field.y, raw_mag_msg->magnetic_field.z);

    raw_imu_pub->publish(std::move(raw_imu_msg));
    raw_mag_pub->publish(std::move(raw_mag_msg));
}