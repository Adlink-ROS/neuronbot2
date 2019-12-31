#include <chrono>
#include <iomanip>
#include <iostream>
#include "neuronbot2_bringup/neuron_serial.hpp"

NeuronSerial::NeuronSerial()
: Node("neuron_serial", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "VADOOT BRRT-A-DEET DA-- Connecting to NeuronBot2 Serial");
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *){});

    // Set up port parameters    
    std::string port = declare_parameter("port", "/dev/neuronbot2");
    int32_t baudrate = declare_parameter("baudrate", 115200);
    std::string cmd_vel_topic = declare_parameter("cmd_vel_topic", "motor_cmd_vel");
    std::string odom_topic = declare_parameter("odom_topic", "odom_data");
    std::string raw_imu_topic = declare_parameter("raw_imu_topic", "raw_imu");
    std::string raw_mag_topic = declare_parameter("raw_mag_topic", "raw_mag");
    odom_frame_parent = declare_parameter("odom_frame_parent", "odom");
    odom_frame_child = declare_parameter("odom_frame_child", "base_footprint");
    imu_frame = declare_parameter("imu_frame", "imu_link");


    // Initialize serial.
    RCLCPP_INFO(get_logger(), "Connecting to serial: '%s', with baudrate '%d'", port.c_str(), baudrate);
    try {
        serial_ = std::make_unique<serial::Serial>(port, baudrate);
        // TODOS: Test timeout
        // serial_ = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(101));
        RCLCPP_INFO(get_logger(), "Connected to serial: '%s', with baudrate '%d', timeout: %d", serial_->getPort().c_str(), serial_->getBaudrate(), serial_->getTimeout());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Beep-bee-bee-boop-bee-doo-weep Can't connect to serail.");
        RCLCPP_FATAL(get_logger(), e.what());
        throw;
    }
    rclcpp::sleep_for(1s);
    frame = std::make_shared<Simple_dataframe>(serial_.get());
    dh = Data_holder::get();
    parameter_init();
    rclcpp::sleep_for(1s);
    read_firmware_info();
    read_robot_parameter();
    rclcpp::sleep_for(1s);
    frame->interact(ID_INIT_ODOM);
    
    // Define Subscriber
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(1), [=](geometry_msgs::msg::Twist::SharedPtr msg) {on_motor_move(msg); });

    // Create Publisher
    odom_pub = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(4));
    raw_imu_pub = create_publisher<sensor_msgs::msg::Imu>(raw_imu_topic, rclcpp::QoS(1));
    raw_mag_pub = create_publisher<sensor_msgs::msg::MagneticField>(raw_mag_topic, rclcpp::QoS(1));
    
    // Set up timer and callback
    keepalive_timer = create_wall_timer(keepalive_period, [=]() { keepalive_cb(); });
    read_timer = create_wall_timer(uart_poll_period, [=]() { 
        update_imu();
        update_odom(); });
}

void NeuronSerial::parameter_init()
{
    // DO NOT even try to change parameters here, or the robot might die. Seriously.
    // I mean seriously.

    // Just don't.

    frame->interact(ID_INIT_ODOM);

    rclcpp::sleep_for(1s);
    
    // Set up control parameter
    Robot_parameter* rp = &dh->parameter;
    memset(rp ,0, sizeof(Robot_parameter));

    rp->params.wheel_diameter = 168;
    rp->params.wheel_track = 288;
    rp->params.encoder_resolution = 3892;
    rp->params.do_pid_interval = 10; 
    rp->params.kp = 75;
    rp->params.ki = 2500;
    rp->params.kd = 0;
    rp->params.ko = 10;
    rp->params.cmd_last_time = 250;
    rp->params.max_v_liner_x = 50;
    rp->params.max_v_liner_y = 0;
    rp->params.max_v_angular_z = 120;
    rp->params.imu_type = 69; // 'E'(69) for enablehttps://bitbucket.org/ROScube/azure_cs_luis/src/master/

    // debug print out
    // RCLCPP_INFO(this->get_logger(),"Request write RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
    //     rp->wheel_diameter, rp->wheel_track,  rp->encoder_resolution, 
    //     rp->do_pid_interval, rp->kp, rp->ki, rp->kd, rp->ko, 
    //     rp->cmd_last_time, rp->max_v_liner_x, rp->max_v_liner_y, rp->max_v_angular_z,
    //     rp->imu_type);
    
    RCLCPP_INFO(this->get_logger(),"Request write RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
        dh->parameter.params.wheel_diameter, 
        dh->parameter.params.wheel_track,  
        dh->parameter.params.encoder_resolution, 
        dh->parameter.params.do_pid_interval, 
        dh->parameter.params.kp, 
        dh->parameter.params.ki, 
        dh->parameter.params.kd, 
        dh->parameter.params.ko, 
        dh->parameter.params.cmd_last_time, 
        dh->parameter.params.max_v_liner_x, 
        dh->parameter.params.max_v_liner_y,
        dh->parameter.params.max_v_angular_z,
        dh->parameter.params.imu_type);
    frame->interact(ID_SET_ROBOT_PARAMTER);
}

void NeuronSerial::read_robot_parameter()
{
    Robot_parameter* rp = &dh->parameter;
    memset(rp, 0, sizeof(Robot_parameter));

    frame->interact(ID_GET_ROBOT_PARAMTER);
 
    RCLCPP_INFO(get_logger(),"RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
        rp->params.wheel_diameter,
        rp->params.wheel_track,  
        rp->params.encoder_resolution, 
        rp->params.do_pid_interval, 
        rp->params.kp, 
        rp->params.ki, 
        rp->params.kd, 
        rp->params.ko, 
        rp->params.cmd_last_time, 
        rp->params.max_v_liner_x, 
        rp->params.max_v_liner_y, 
        rp->params.max_v_angular_z,
        rp->params.imu_type);
}

void NeuronSerial::read_firmware_info()
{
    frame->interact(ID_GET_VERSION);
    std::string version(dh->firmware_info.version);
    std::string time(dh->firmware_info.time);
    RCLCPP_INFO(get_logger(), "Robot firmware version:%s || build time:%s", version.c_str(), time.c_str());

}

void NeuronSerial::on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "Serial receives cmd_vel ( %f, %f, %f )", msg->linear.x, msg->linear.y, msg->angular.z);
    dh->velocity.v_liner_x = msg->linear.x * 100;
    dh->velocity.v_liner_y = msg->linear.x * 100;
    dh->velocity.v_angular_z = msg->angular.z * 100;
    frame->interact(ID_SET_VELOCITY);
    RCLCPP_INFO(this->get_logger(), "Message sent");
}

void NeuronSerial::keepalive_cb()
{
    // Don't know if this callback is necessary.
    RCLCPP_INFO(get_logger(), "Do random things to keep serial alive");
    frame->interact(ID_GET_VERSION);
}

void NeuronSerial::update_odom()
{
    frame->interact(ID_GET_ODOM);
    RCLCPP_INFO(get_logger(), "1");
    auto now = get_clock()->now();
    float x = dh->odom.x*0.01;
    float y = dh->odom.y*0.01;
    float th = dh->odom.yaw*0.01;

    float vxy = dh->odom.v_liner_x*0.01;
    float vth = dh->odom.v_angular_z*0.01;

    RCLCPP_INFO(get_logger(), "odom: x=%.2f y=%.2f th=%.2f vxy=%.2f vth=%.2f", x, y ,th, vxy,vth);
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th);

    //publish the message  
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = odom_frame_parent;
    odom->child_frame_id = odom_frame_child;
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
    
    RCLCPP_INFO(get_logger(), "Pub odom");
    odom_pub->publish(std::move(odom));
}

void NeuronSerial::update_imu()
{
    frame->interact(ID_GET_IMU_DATA);

    const rclcpp::Time & now = get_clock()->now();
    auto raw_imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    raw_imu_msg->header.frame_id = imu_frame;
    raw_imu_msg->header.stamp = now;    
    raw_imu_msg->linear_acceleration.x = dh->imu_data[0];
    raw_imu_msg->linear_acceleration.y = dh->imu_data[1];
    raw_imu_msg->linear_acceleration.z= dh->imu_data[2];
    raw_imu_msg->angular_velocity.x = dh->imu_data[3];
    raw_imu_msg->angular_velocity.y = dh->imu_data[4];
    raw_imu_msg->angular_velocity.z = dh->imu_data[5];

    auto raw_mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
    raw_mag_msg->header.frame_id = imu_frame;
    raw_mag_msg->header.stamp = now;
    raw_mag_msg->magnetic_field.x = dh->imu_data[6];
    raw_mag_msg->magnetic_field.y = dh->imu_data[7];
    raw_mag_msg->magnetic_field.z = dh->imu_data[8];
    
    RCLCPP_INFO(this->get_logger(), "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", raw_imu_msg->linear_acceleration.x, raw_imu_msg->linear_acceleration.y, raw_imu_msg->linear_acceleration.z,
    raw_imu_msg->angular_velocity.x, raw_imu_msg->angular_velocity.y ,raw_imu_msg->angular_velocity.z,
    raw_mag_msg->magnetic_field.x, raw_mag_msg->magnetic_field.y, raw_mag_msg->magnetic_field.z);

    raw_imu_pub->publish(std::move(raw_imu_msg));
    raw_mag_pub->publish(std::move(raw_mag_msg));
}