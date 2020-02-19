#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "behaviortree_cpp_v3/action_node.h"

// Custom type
struct Pose2D
{
    double x, y, theta;
};


namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}
} // end namespace BT


class Nav2Client : public BT::AsyncActionNode
{
public:
    Nav2Client(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose2D>("goal")};
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("nav2_client");
	    auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "NavigateToPose");
        // if no server is present, fail after 5 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
        		// RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        		return BT::NodeStatus::FAILURE;
        }
        // Take the goal from the InputPort of the Node
        Pose2D goal;
        if (!getInput<Pose2D>("goal", goal)) {
          // if I can't get this, there is something wrong with your BT.
          // For this reason throw an exception instead of returning FAILURE
          throw BT::RuntimeError("missing required input [goal]");
        }

        _aborted = false;
        printf("%.2f, %.2f, %.2f", goal.x, goal.y, goal.theta);
        RCLCPP_INFO(node_->get_logger(), "Sending goal %f %f %f", goal.x, goal.y, goal.theta);

	    nav2_msgs::action::NavigateToPose::Goal goal_msg;
	    goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->get_clock()->now();
	    goal_msg.pose.pose.position.x = goal.x;
	    goal_msg.pose.pose.position.y = goal.y;
	    goal_msg.pose.pose.position.z = 0.0;
            tf2::Quaternion rot;
            rot.setRPY(0, 0, goal.theta);
	    rot.normalize();
	    geometry_msgs::msg::Quaternion quat_msg;
	    quat_msg = tf2::toMsg(rot);
	    goal_msg.pose.pose.orientation.x = quat_msg.x;
	    goal_msg.pose.pose.orientation.y = quat_msg.y;
	    goal_msg.pose.pose.orientation.z = quat_msg.z;
	    goal_msg.pose.pose.orientation.w = quat_msg.w;
        

	    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    	if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    			rclcpp::executor::FutureReturnCode::SUCCESS)
    	{
    		RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
    		return BT::NodeStatus::FAILURE;
    	}

	    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle = goal_handle_future.get();
	    if (!goal_handle) {
	    	RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    		return BT::NodeStatus::FAILURE;
	    }

	    auto result_future = action_client->async_get_result(goal_handle);

	    RCLCPP_INFO(node_->get_logger(), "Waiting for result");
	    if (rclcpp::spin_until_future_complete(node_, result_future) !=
	    		rclcpp::executor::FutureReturnCode::SUCCESS)
	    {
	    	RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
    		return BT::NodeStatus::FAILURE;
	    }

	    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

	    switch (wrapped_result.code) {
       		case rclcpp_action::ResultCode::SUCCEEDED:
         			break;
	    	case rclcpp_action::ResultCode::ABORTED:
        			RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
    		        return BT::NodeStatus::FAILURE;
       		case rclcpp_action::ResultCode::CANCELED:
         			RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
    		        return BT::NodeStatus::FAILURE;
       		default:
         			RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
    		        return BT::NodeStatus::FAILURE;
     	}

        if (_aborted) {
          // this happens only if method halt() was invoked
          //_client.cancelAllGoals();
          RCLCPP_INFO(node_->get_logger(), "MoveBase aborted");
          return BT::NodeStatus::FAILURE;
        }

	    RCLCPP_INFO(node_->get_logger(), "result received");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
        _aborted = true;
    }
private:
    bool _aborted;
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};
