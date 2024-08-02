#include "Robot.h"

Robot::ArticulatedRobot::ArticulatedRobot() : 
    spinner(1), 
    move_group_interface("bdr_ur10"), 
    visual_tools("base_link")
{
    // Initialize ROS spinner
    spinner.start();

    // Load MoveIt Visual Tools
    visual_tools.loadRemoteControl();
    visual_tools.deleteAllMarkers();

    // Set the text pose for the RViz visual tools
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.25; // Position the text above the robot base

    // Display initial messages
    visual_tools.publishText(text_pose, "MoveIt Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();

    // Set the maximum planning time
    move_group_interface.setPlanningTime(10.0); // Increase planning time to 10 seconds
}

void Robot::ArticulatedRobot::PTP(geometry_msgs::Pose target)
{
    // Visualize the target pose
    visual_tools.publishAxisLabeled(target, "target_pose");
    visual_tools.trigger(); // Trigger to display changes in RViz

    // Set the pose target for MoveIt
    move_group_interface.setPoseTarget(target);

    // Plan and move to the target
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_interface.move();
    }
    else
    {
        ROS_WARN("Planning failed. Unable to move to target pose.");
    }
}

void Robot::ArticulatedRobot::planAndVisualize(geometry_msgs::Pose target)
{
    // Visualize the target pose
    visual_tools.publishAxisLabeled(target, "target_pose");
    visual_tools.trigger(); // Trigger to display changes in RViz

    // Set the pose target for MoveIt
    move_group_interface.setPoseTarget(target);

    // Plan and move to the target
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_interface.move();
    }
    else
    {
        ROS_WARN("Planning failed. Unable to move to target pose.");
    }
}
