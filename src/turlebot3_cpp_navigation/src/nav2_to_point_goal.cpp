#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "t3_action_msg/action/move.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class NavToPoseClient : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    explicit NavToPoseClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) 
    : Node{"nav_to_pose_client", node_options}, goal_done_(false){
        // Inizializza l'Action Client
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigate_to_pose");
                // Attendi che l'Action Server sia disponibile
        while (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavToPoseClient::send_goal, this));
    }
    bool is_goal_done() const{
        return this->goal_done_;
    }
    // Funzione per inviare il goal
    void send_goal(float x, float y, float theta){
        // Crea un goal
        auto goal_msg = NavigateToPose::Goal();
           // Create a PoseStamped message
        geometry_msgs::msg::PoseStamped pose;
    }
    private:
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool goal_done_;
};