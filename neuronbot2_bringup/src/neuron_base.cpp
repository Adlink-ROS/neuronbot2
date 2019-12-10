#include "neuronbot2_bringup/neuron_base.hpp"

NeuronBase::NeuronBase()
: Node("neuron_base", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "BAWHOOOP~ Initiating Neuronbot2 Base Driver");

    // Declare parameters
    double update_odom_freq = declare_parameter("update_odom_freq", 10.0);

    // Define timer
    update_odom_timer = create_wall_timer(1s/ update_odom_freq, [=]() {update_odom(); });

    // Define subscriber
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(1), [=](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_cmd_vel(msg); });
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(4), [=](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(msg); });
    // Define publisher
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("motor_cmd_vel", rclcpp::QoS(1));
    // tf_broadcaster_ = create_publisher<tf2_ros::TransformBroadcaster>("tf", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void NeuronBase::on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    motor_cmd.linear.x = msg->linear.x;
    motor_cmd.linear.y = msg->linear.y;
    motor_cmd.angular.z = msg->angular.z; 

    RCLCPP_INFO(get_logger(), "Update cmd_vel x = %f, y = %f, theta = %f", motor_cmd.linear.x, motor_cmd.linear.y, motor_cmd.angular.z);
}

void NeuronBase::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    odom_tf.header.stamp = get_clock()->now();
    odom_tf.header.frame_id = msg->header.frame_id;
    odom_tf.child_frame_id = msg->child_frame_id;
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;
    RCLCPP_INFO(get_logger(), "Update TF from %s to %s", odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str());
}

void NeuronBase::update_odom()
{
    RCLCPP_INFO(get_logger(), "pub cmd_vel x = %f, y = %f, theta = %f", motor_cmd.linear.x, motor_cmd.linear.y, motor_cmd.angular.z);
    cmd_vel_pub->publish(motor_cmd);
    RCLCPP_INFO(get_logger(), "pub TF from %s to %s", odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str());
    if (publish_tf_)
        tf_broadcaster_->sendTransform(odom_tf);
}