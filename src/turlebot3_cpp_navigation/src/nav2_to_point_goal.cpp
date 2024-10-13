/**
 * @file nav2_to_point_goal.cpp
 * @brief Implementation of a ROS 2 node that sends navigation goals to a robot.
 *
 * This file contains the implementation of the NavToPoseClient class, which is a ROS 2 node that
 * sends navigation goals to a robot using the NavigateToPose action from the nav2_msgs package.
 * The node initializes an action client, waits for the action server to be available, and sends
 * goals to the server. It also handles goal responses, feedback, and results.
 *
 * Dependencies:
 * - rclcpp
 * - rclcpp_action
 * - nav2_msgs
 * - geometry_msgs
 * - tf2
 * - tf2_geometry_msgs
 *
 * @author Valerio Passamano
 */

#include "nav2_to_point_goal.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

NavToPoseClient::NavToPoseClient(const rclcpp::NodeOptions & node_options)
    : Node{"nav_to_pose_client", node_options}, goal_done_(false) {

    // Initialize the Action Client
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        this->get_node_base_interface(),  // Base interface for the node
        this->get_node_graph_interface(), // Graph interface for the node
        this->get_node_logging_interface(), // Logging interface for the node
        this->get_node_waitables_interface(), // Waitables interface for the node
        "navigate_to_pose" // Name of the action
    );

    // Wait for the Action Server to be available
    while (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        // Log a message while waiting for the action server
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    // Create a timer to periodically send goals
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), // Timer interval set to 500ms
        std::bind(&NavToPoseClient::send_goal, this) // Bind the send_goal method to the timer
    );
}

void NavToPoseClient::goal_response_callback(const std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
    // Get the goal handle from the future object
    auto goal_handle = future.get();
    if (!goal_handle) {
        // Log an error message if the goal was rejected by the server
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        // Log an info message if the goal was accepted by the server
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void NavToPoseClient::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    // Log the current position of the robot from the feedback message
    RCLCPP_INFO(this->get_logger(), "Current robot position: (%.2f, %.2f)",
                feedback->current_pose.pose.position.x,
                feedback->current_pose.pose.position.y);
}

void NavToPoseClient::result_callback(const GoalHandleNavigateToPose::WrappedResult& result) {
    // Check the result code and log the appropriate message
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // Log an info message if the goal was reached successfully
            RCLCPP_INFO(this->get_logger(), "Goal was reached");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            // Log an error message if the goal was aborted
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            // Log an error message if the goal was canceled
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            // Log an error message for any unknown result code
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
    // Set the goal_done_ flag to true indicating the goal is done
    this->goal_done_ = true;
}

bool NavToPoseClient::is_goal_done() const {
    // Return the status of the goal
    return this->goal_done_;
}

void NavToPoseClient::send_goal(float x, float y, float theta) {
    // Stop the timer after the first call
    this->timer_->cancel();

    // Reset the goal status
    this->goal_done_ = false;

    // Check if the action client is initialized
    if (!this->client_ptr_) {
        // Log an error message if the action client is not initialized
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        return;
    }

    // Wait for the action server to be available
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        // Log an error message if the action server is not available after waiting
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done_ = true;
        return;
    }

    // Create a goal message
    auto goal_msg = NavigateToPose::Goal();
    // Create a PoseStamped message using the create_pose_stamped function
    goal_msg.pose = create_pose_stamped(x, y, theta);

    // Log an info message with the goal position
    RCLCPP_INFO(this->get_logger(), "Sending goal to position (x: %.2f, y: %.2f, yaw: %.2f)", 
                x, y, theta);

    // Set up the goal options with the appropriate callbacks
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&NavToPoseClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&NavToPoseClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NavToPoseClient::result_callback, this, std::placeholders::_1);

    // Send the goal asynchronously
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

geometry_msgs::msg::PoseStamped NavToPoseClient::create_pose_stamped(double position_x, double position_y, double orientation_z) {
    // Create a PoseStamped message
    geometry_msgs::msg::PoseStamped pose;

    // Set the frame_id to 'map' (global frame)
    pose.header.frame_id = "map";

    // Get the current time for the timestamp
    pose.header.stamp = this->get_clock()->now();

    // Set the position coordinates
    pose.pose.position.x = position_x;
    pose.pose.position.y = position_y;
    pose.pose.position.z = 0.0;  // Assuming no height (2D navigation)

    // Convert Euler angles (roll = 0.0, pitch = 0.0, yaw = orientation_z) to quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, orientation_z);

    // Set the orientation using the quaternion
    pose.pose.orientation = tf2::toMsg(quaternion);
    
    // Return the PoseStamped message
    return pose;
}
