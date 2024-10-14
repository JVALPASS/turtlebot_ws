/**
 * @file nav2_to_point_goal.hpp
 * @brief Declaration of the NavToPoseClient class.
 *
 * This file contains the declaration of the NavToPoseClient class, which is a ROS 2 node that
 * sends navigation goals to a robot using the NavigateToPose action from the nav2_msgs package.
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

#ifndef NAV2_TO_POINT_GOAL_HPP
#define NAV2_TO_POINT_GOAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>

class NavToPoseClient : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Construct a new NavToPoseClient object.
     * 
     * This constructor initializes the ROS 2 node, sets up the action client, and waits for the action server to be available.
     * It also sets up a timer to periodically send goals.
     * 
     * @param node_options Options for configuring the node.
     */
    explicit NavToPoseClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

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
     * 
     * @param result The result of the goal execution.
     */
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

    /**
     * @brief Check if the goal is done.
     * 
     * This function returns the status of the goal.
     * 
     * @return true if the goal is done, false otherwise.
     */
    bool is_goal_done() const;

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

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    bool goal_done_;
};

#endif  // NAV2_TO_POINT_GOAL_HPP
