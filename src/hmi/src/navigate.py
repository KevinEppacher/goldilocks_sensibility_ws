#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

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

def main():
    # Initialize ROS node
    rospy.init_node('move_dual_manipulators', anonymous=True)
    roscpp_initialize([])

    # Create MoveGroupCommanders for both groups
    quantec_group = MoveGroupCommander('quantec_manipulator')
    ur10_group = MoveGroupCommander('ur10_manipulator')

    # Define the goal pose for quantec_manipulator
    quantec_goal_pose = Pose()
    quantec_goal_pose.position.x = 0.450247
    quantec_goal_pose.position.y = 0.374964
    quantec_goal_pose.position.z = 2.482706
    quantec_quaternion = quaternion_from_euler(-2.183697, 0.359914, 0.017947)  # Roll, Pitch, Yaw
    quantec_goal_pose.orientation.x = quantec_quaternion[0]
    quantec_goal_pose.orientation.y = quantec_quaternion[1]
    quantec_goal_pose.orientation.z = quantec_quaternion[2]
    quantec_goal_pose.orientation.w = quantec_quaternion[3]

    # Define the goal pose for ur10_manipulator
    ur10_goal_pose = Pose()
    ur10_goal_pose.position.x = 0.394390
    ur10_goal_pose.position.y = -0.783308
    ur10_goal_pose.position.z = 0.957069
    ur10_quaternion = quaternion_from_euler(1.570915, -0.014215, 1.571423)  # Roll, Pitch, Yaw
    ur10_goal_pose.orientation.x = ur10_quaternion[0]
    ur10_goal_pose.orientation.y = ur10_quaternion[1]
    ur10_goal_pose.orientation.z = ur10_quaternion[2]
    ur10_goal_pose.orientation.w = ur10_quaternion[3]

    # Move quantec_manipulator to its goal pose
    rospy.loginfo("Moving quantec_manipulator to its goal pose...")
    move_to_goal(quantec_goal_pose, quantec_group)

    # Move ur10_manipulator to its goal pose
    rospy.loginfo("Moving ur10_manipulator to its goal pose...")
    move_to_goal(ur10_goal_pose, ur10_group)

    # Shutdown MoveIt Commander
    roscpp_shutdown()

if __name__ == '__main__':
    main()
