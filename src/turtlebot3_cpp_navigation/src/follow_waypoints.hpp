/**
 * @file follow_waypoints.hpp
 * @brief Declaration of the FollowWaypointsClient class.
 *
 * This file contains the declaration of the FollowWaypointsClient class, which is a ROS 2 node that
 * sends a series of navigation goals (waypoints) to a robot using the NavigateToPose action from the nav2_msgs package.
 * The class defines methods for sending goals, handling feedback, and processing the result.
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

#ifndef FOLLOW_WAYPOINTS_HPP
#define FOLLOW_WAYPOINTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <vector>

class FollowWaypointsClient : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Construct a new FollowWaypointsClient object.
     * 
     * This constructor initializes the ROS 2 node, sets up the action client, and waits for the action server to be available.
     * 
     * @param node_options Options for configuring the node.
     */
    explicit FollowWaypointsClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

    /**
     * @brief Callback function for goal response.
     * 
     * This function is called when the action server responds to the goal request.
     * It checks if the goal was accepted or rejected by the server.
     * 
     * @param future A future object containing the goal handle.
     */
    void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle);

    /**
     * @brief Callback function for feedback.
     * 
     * This function is called periodically by the action server to provide feedback on the goal execution.
     * It logs the current position of the robot.
     * 
     * @param goal_handle The goal handle.
     * @param feedback A shared pointer to the feedback message.
     */
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /**
     * @brief Callback function for result.
     * 
     * This function is called when the action server completes the goal execution.
     * It checks the result code and logs the outcome of the goal.
     * If the goal succeeds, the client proceeds to the next waypoint.
     * 
     * @param result The result of the goal execution.
     */
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

    /**
     * @brief Send a series of navigation goals to the robot (waypoints).
     * 
     * This function sends a series of goals to the robot to follow a series of waypoints.
     * 
     * @param waypoints A vector of waypoints where each waypoint is represented by a tuple of (x, y, theta).
     */
    void send_waypoints(const std::vector<std::tuple<float, float, float>>& waypoints);
    
        /**
     * @brief Send a navigation goal to the robot.
     * 
     * This function creates a goal message with the specified position and orientation,
     * and sends it to the action server.
     * 
     * @param x The x-coordinate of the goal position.
     * @param y The y-coordinate of the goal position.
     * @param theta The orientation (yaw) of the goal position.
     */
    void send_goal(float x, float y, float theta);


private:
    /**
     * @brief Create a PoseStamped message.
     * 
     * This function creates a PoseStamped message with the specified position and orientation.
     * 
     * @param position_x The x-coordinate of the position.
     * @param position_y The y-coordinate of the position.
     * @param orientation_z The orientation (yaw) of the position.
     * @return A PoseStamped message.
     */
    geometry_msgs::msg::PoseStamped create_pose_stamped(double position_x, double position_y, double orientation_z);

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    bool goal_done_;
    size_t current_waypoint_;
    std::vector<std::tuple<float, float, float>> waypoints_;
};

#endif  // FOLLOW_WAYPOINTS_HPP
