#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot3_py_navigation.navigation_helper import NavigationHelper

class FollowWaypoints(Node):
    def __init__(self):
        super().__init__("follow_waypoints")
        self.helper = NavigationHelper()
    
    def follow_waypoints(self):
        # --- Follow waypoints
        goal_pose1 = self.helper.create_pose_stamped(2.5, 1.0, 1.57)
        goal_pose2 = self.helper.create_pose_stamped(2.0, 2.5, 3.14)
        goal_pose3 = self.helper.create_pose_stamped(0.0, 0.0, 0.0)

        waypoints = [goal_pose1, goal_pose2, goal_pose3]
        self.helper.navigator.followWaypoints(waypoints)
        
        while not self.helper.navigator.isTaskComplete():
            feedback = self.helper.navigator.getFeedback()
            print(feedback)
            pass  # This keeps the loop running until the task is complete
    
        print(self.helper.navigator.getResult())
def main():
    # --- Init
    rclpy.init()

    node = FollowWaypoints()
    node.follow_waypoints()
    # --- Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()