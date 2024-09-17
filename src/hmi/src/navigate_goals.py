#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseArray

def move_to_goal(pose, move_group):
    """
    Moves the specified planning group to the given pose.
    """
    move_group.set_pose_target(pose)  # Set the target pose

    # Plan the trajectory
    plan = move_group.plan()

    # Check if the plan is a tuple and handle it appropriately
    if isinstance(plan, tuple):
        plan = plan[1]  # Extract the actual plan if returned as a tuple

    # Execute the plan if it is valid
    if plan:
        execution_success = move_group.execute(plan, wait=True)
        if execution_success:
            rospy.loginfo(f"Successfully moved {move_group.get_name()} to the goal pose.")
        else:
            rospy.logwarn(f"Failed to execute the trajectory for {move_group.get_name()}.")
    else:
        rospy.logwarn(f"Failed to plan for the goal pose for {move_group.get_name()}.")

def measurement_points_callback(msg):
    """
    Callback function to handle PoseArray messages from the measurement_points topic.
    Moves the ur10_manipulator to each pose in sequence.
    """
    rospy.loginfo("Received new measurement points.")

    # Create MoveGroupCommander for ur10_manipulator
    ur10_group = MoveGroupCommander('ur10_manipulator')

    # Iterate over each pose in the received PoseArray
    for i, goal_pose in enumerate(msg.poses):
        rospy.loginfo(f"Moving ur10_manipulator to goal {i + 1}...")

        # Move ur10_manipulator to its goal pose
        move_to_goal(goal_pose, ur10_group)

def main():
    # Initialize ROS node
    rospy.init_node('move_ur10_manipulator', anonymous=True)
    roscpp_initialize([])

    # Subscribe to the measurement_points topic to receive goal poses
    rospy.Subscriber("measurement_points", PoseArray, measurement_points_callback)

    rospy.loginfo("Waiting for measurement points...")
    rospy.spin()  # Keep the node alive to continue receiving messages

    # Shutdown MoveIt Commander
    roscpp_shutdown()

if __name__ == '__main__':
    main()
