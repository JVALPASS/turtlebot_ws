#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, posistion_x, posistion_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = posistion_x
    pose.pose.position.y = posistion_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    # --- Init
    rclpy.init()
    
    nav = BasicNavigator()

    # --- Follow waypoints
    goal_pose1 = create_pose_stamped(nav, 2.5, 1.0, 1.57)
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, 1.57)

    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)
    
    while not nav.isTaskComplete():
        pass  # This keeps the loop running until the task is complete
        #feedback = nav.getFeedback()
        #print(feedback)
    
    print(nav.getResult())

    # --- Shutdown
    rclpy.shutdown()

if __name__ =='__main__':
    main()