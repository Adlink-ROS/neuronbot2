#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("send_goal_client");
	auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "NavigateToPose");

	if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
		RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
		return 1;
	}

	nav2_msgs::action::NavigateToPose::Goal goal_msg;
	goal_msg.pose.header.frame_id = "map";

	goal_msg.pose.pose.position.x = -1.5;
	goal_msg.pose.pose.position.y = -1.5;
	goal_msg.pose.pose.position.z = 0.0;
	goal_msg.pose.pose.orientation.x = 0;
	goal_msg.pose.pose.orientation.y = 0;
	goal_msg.pose.pose.orientation.z = 0;
	goal_msg.pose.pose.orientation.w = 1;

	RCLCPP_INFO(node->get_logger(), "sending goal");
	auto goal_handle_future = action_client->async_send_goal(goal_msg);
	if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
			rclcpp::executor::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node->get_logger(), "send goal call failed : (");
		return 1;
	}

	rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle = goal_handle_future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
		return 1;
	}

	auto result_future = action_client->async_get_result(goal_handle);

	RCLCPP_INFO(node->get_logger(), "Waiting for result");
	if (rclcpp::spin_until_future_complete(node, result_future) !=
			rclcpp::executor::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node->get_logger(), "get result call failed :  (" );
		return 1;
	}

	rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

	switch (wrapped_result.code) {
    		case rclcpp_action::ResultCode::SUCCEEDED:
      			break;
		case rclcpp_action::ResultCode::ABORTED:
     			RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      			return 1;
    		case rclcpp_action::ResultCode::CANCELED:
      			RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      			return 1;
    		default:
      			RCLCPP_ERROR(node->get_logger(), "Unknown result code");
     			return 1;
  	}
	  
	RCLCPP_INFO(node->get_logger(), "result received");

  	rclcpp::shutdown();
  	return 0;
}
