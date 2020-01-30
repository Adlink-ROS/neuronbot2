#include "neuronbot2_bringup/neuron_base.hpp"

using namespace neuronbot2;

NeuronBase::NeuronBase()
: Node("neuron_base", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "BAWHOOOP~ Initiating Neuronbot2 Base Driver");

    // Declare parameters
    std::string cmd_vel_sub_topic = declare_parameter("cmd_vel_sub_topic", "cmd_vel");
    std::string cmd_vel_repub_topic = declare_parameter("cmd_vel_repub_topic", "motor_cmd_vel");
    std::string raw_imu_sub_topic = declare_parameter("raw_imu_sub_topic", "raw_imu");
    std::string imu_repub_topic = declare_parameter("imu_repub_topic", "imu");
    std::string odom_sub_topic = declare_parameter("odom_sub_topic", "raw_odom");

    // imu calibration
    acc_x_bias = declare_parameter("acc_x_bias", -2.42);
    acc_y_bias = declare_parameter("acc_y_bias", 0.19);
    acc_z_bias = declare_parameter("acc_z_bias", -2.0);
    vel_theta_bias = declare_parameter("vel_theta_bias", 0.012);

    // Function switch
    publish_tf_ = declare_parameter("publish_tf", false);
    calibrate_imu_ = declare_parameter("calibrate_imu", true);

    // Define timer
    double update_odom_freq = declare_parameter("update_odom_freq", 10.0);
    update_odom_timer = create_wall_timer(1s/ update_odom_freq, [=]() {update_odom(); });

    // Define subscriber
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
       cmd_vel_sub_topic , rclcpp::QoS(1), 
       [=](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_cmd_vel(msg); });
    raw_imu_sub = create_subscription<sensor_msgs::msg::Imu>(
       raw_imu_sub_topic , rclcpp::QoS(1), 
       [=](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_raw_imu(msg); });
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        odom_sub_topic, rclcpp::QoS(4), 
        [=](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(msg); });

    // Define publisher
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>( 
        cmd_vel_repub_topic, rclcpp::QoS(1));
    imu_pub = create_publisher<sensor_msgs::msg::Imu>( 
        imu_repub_topic, rclcpp::QoS(1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void NeuronBase::on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    // Subscribe to cmd_vel from ROS command, and re-publish it to serial in static frequency. 
    motor_cmd.linear.x = msg->linear.x;
    motor_cmd.linear.y = msg->linear.y;
    motor_cmd.angular.z = msg->angular.z; 

    RCLCPP_DEBUG(get_logger(), "Update cmd_vel x = %f, y = %f, theta = %f", motor_cmd.linear.x, motor_cmd.linear.y, motor_cmd.angular.z);
}

void NeuronBase::on_raw_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    // Calibrate the data in "raw_imu", and re-publish it to "imu"
    imu_data_ = *msg;
    imu_data_.angular_velocity.z = msg->angular_velocity.z - vel_theta_bias;
    imu_data_.linear_acceleration.x = msg->linear_acceleration.x - acc_x_bias;
    imu_data_.linear_acceleration.y = msg->linear_acceleration.y - acc_y_bias;
    imu_data_.linear_acceleration.z = msg->linear_acceleration.z - acc_z_bias;
}
void NeuronBase::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // Subscribe to odom from serail, and re-publish it to the higher level.
    odom_tf.header.stamp = msg->header.stamp; 
    odom_tf.header.frame_id = msg->header.frame_id;
    odom_tf.child_frame_id = msg->child_frame_id;
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;
    RCLCPP_DEBUG(get_logger(), "Update TF from %s to %s", odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str());
}

void NeuronBase::update_odom()
{
    RCLCPP_DEBUG(get_logger(), "pub cmd_vel x = %f, y = %f, theta = %f", motor_cmd.linear.x, motor_cmd.linear.y, motor_cmd.angular.z);
    cmd_vel_pub->publish(motor_cmd);
    RCLCPP_DEBUG(get_logger(), "pub TF from %s to %s", odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str());
    if (publish_tf_)
        tf_broadcaster_->sendTransform(odom_tf);
    if (calibrate_imu_)
        imu_pub->publish(imu_data_);    
}
