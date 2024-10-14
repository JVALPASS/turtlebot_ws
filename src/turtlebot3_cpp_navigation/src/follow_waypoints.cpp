#include "follow_waypoints.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

FollowWaypointsClient::FollowWaypointsClient(const rclcpp::NodeOptions& node_options)
    : Node{"follow_waypoints_client", node_options}, goal_done_{false}, current_waypoint_{0} {

    // Initialize the Action Client
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        this->get_node_base_interface(),  // Base interface for the node
        this->get_node_graph_interface(), // Graph interface for the node
        this->get_node_logging_interface(), // Logging interface for the node
        this->get_node_waitables_interface(), // Waitables interface for the node
        "navigate_to_pose" // Name of the action
    );

    // Ensure that the client pointer is valid before proceeding
    if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create action client");
        throw std::runtime_error("Failed to create action client");
    }

    // Wait for the Action Server to be available
    while (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    // Check if action server is available before continuing
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        throw std::runtime_error("Action server not available");
    }
}

void FollowWaypointsClient::goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void FollowWaypointsClient::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    if (!feedback) {
        RCLCPP_ERROR(this->get_logger(), "Received null feedback pointer");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Current robot position: (%.2f, %.2f)",
                feedback->current_pose.pose.position.x,
                feedback->current_pose.pose.position.y);
}

void FollowWaypointsClient::result_callback(const GoalHandleNavigateToPose::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %lu", current_waypoint_);
        current_waypoint_++;

        if (current_waypoint_ < waypoints_.size()) {
            // Proceed to the next waypoint
            auto next_waypoint = waypoints_[current_waypoint_];
            send_goal(std::get<0>(next_waypoint), std::get<1>(next_waypoint), std::get<2>(next_waypoint));
        } else {
            RCLCPP_INFO(this->get_logger(), "All waypoints completed");
        }
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    }
    this->goal_done_ = true;
}

void FollowWaypointsClient::send_waypoints(const std::vector<std::tuple<float, float, float>>& waypoints) {
    if (waypoints.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No waypoints provided");
        return;
    }

    this->waypoints_ = waypoints;
    current_waypoint_ = 0;

    auto first_waypoint = waypoints_[current_waypoint_];
    send_goal(std::get<0>(first_waypoint), std::get<1>(first_waypoint), std::get<2>(first_waypoint));
}

void FollowWaypointsClient::send_goal(float x, float y, float theta) {
    // Reset goal status
    this->goal_done_ = false;

    // Check if the action client is initialized
    if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        return;
    }

    // Wait for the action server to be available
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done_ = true;
        return;
    }

    // Create a goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = create_pose_stamped(x, y, theta);

    RCLCPP_INFO(this->get_logger(), "Sending goal to position (x: %.2f, y: %.2f, yaw: %.2f)", x, y, theta);

    // Set up the goal options with callbacks
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FollowWaypointsClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&FollowWaypointsClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&FollowWaypointsClient::result_callback, this, std::placeholders::_1);

    // Send the goal asynchronously
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

geometry_msgs::msg::PoseStamped FollowWaypointsClient::create_pose_stamped(double position_x, double position_y, double orientation_z) {
    geometry_msgs::msg::PoseStamped pose;

    // Set the frame_id and timestamp
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();

    // Set the position
    pose.pose.position.x = position_x;
    pose.pose.position.y = position_y;
    pose.pose.position.z = 0.0;  // 2D navigation

    // Set the orientation using a quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, orientation_z);  // Yaw only
    pose.pose.orientation = tf2::toMsg(quaternion);

    return pose;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    auto follow_waypoints_client = std::make_shared<FollowWaypointsClient>(node_options);

    // Define waypoints (x, y, theta)
    std::vector<std::tuple<float, float, float>> waypoints = {
        {1.0, 1.0, 0.0},
        {2.0, 2.0, 1.57},
        {3.0, 1.5, 3.14}
    };

    // Send waypoints to the robot
    follow_waypoints_client->send_waypoints(waypoints);

    // Spin the node to process callbacks
    rclcpp::spin(follow_waypoints_client);

    rclcpp::shutdown();
    return 0;
}
