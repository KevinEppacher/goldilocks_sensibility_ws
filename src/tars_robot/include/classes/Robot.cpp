#include "Robot.h"

Robot::ArticulatedRobot::ArticulatedRobot(): 
    spinner(1), 
    move_group_interface("bdr_ur10"), 
    visual_tools("base_link")
{
    
}

void Robot::ArticulatedRobot::PTP(geometry_msgs::Pose target)
{

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    Robot::ArticulatedRobot::planAndVisualize(target);
}

void Robot::ArticulatedRobot::planAndVisualize(geometry_msgs::Pose target)
{
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = target.position.x;
    target_pose1.position.y = target.position.y;
    target_pose1.position.z = target.position.z;

    tf2::Quaternion orientation;
    orientation.setRPY(3.14 * target.orientation.x /180, 3.14 * target.orientation.y /180, 3.14 * target.orientation.z /180);
    target_pose1.orientation.w = orientation.getW();
    target_pose1.orientation.x = orientation.getX();
    target_pose1.orientation.y = orientation.getY();
    target_pose1.orientation.z = orientation.getZ();
    move_group_interface.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_interface.move();
    }
}