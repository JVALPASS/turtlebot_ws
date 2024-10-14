#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot3_py_navigation.navigation_helper import NavigationHelper

class InitialPoseNode(Node):
    def __init__(self):
        super().__init__("initial_pose_node")
        self.helper = NavigationHelper()

    def set_initial_pose(self):
        initial_pose = self.helper.create_pose_stamped(0.0, 0.0, 0.0)

        self.helper.navigator.setInitialPose(initial_pose)

        # --- Wait for Nav2
        self.helper.navigator.waitUntilNav2Active()

def main(args=None):
    # --- Init
    rclpy.init(args=args)
    node = InitialPoseNode()
    node.set_initial_pose()
    # --- Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()