#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot3_py_navigation.navigation_helper import NavigationHelper

class NavigationToGoal(Node):
    def __init__(self):
        super().__init__("navigation_to_goal")
        self.helper = NavigationHelper()
    
    def nav_to_goal(self):
        # --- Send Nav2 goal
        # PI == 3.14 == 180
        # PI/2 == 1.57 == 90
        goal_pose = self.helper.create_pose_stamped(2.5, 1.0, 1.57)
        self.helper.navigator.goToPose(goal_pose)
        while not self.helper.navigator.isTaskComplete():
            pass  # This keeps the loop running until the task is complete
            #feedback = nav.getFeedback()
            #print(feedback)
        print(self.helper.navigator.getResult())

def main():
    # --- Init
    rclpy.init()
    node = NavigationToGoal()
    node.nav_to_goal()
    # --- Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()