#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <string>
#include <iostream>

namespace Robot {
    class ArticulatedRobot{
        public:
            ArticulatedRobot();
            void PTP(geometry_msgs::Pose target);
            void setupVisualisation();
            void planAndVisualize(geometry_msgs::Pose target);


        private:
            std::string planningGroup;
            ros::AsyncSpinner spinner;
            moveit::planning_interface::MoveGroupInterface move_group_interface;
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            moveit_visual_tools::MoveItVisualTools visual_tools;
            Eigen::Isometry3d text_pose;

            float x, y, z, roll, pitch, yaw;
    };
}


/*
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class MoveGroupInterfaceTutorial
{
public:
  MoveGroupInterfaceTutorial()
    : spinner(1), move_group_interface("manipulator"), visual_tools("base_link")
  {
  }

  void run()
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    setupVisualization();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    planAndVisualize();
  }

private:
  void setupVisualization()
  {
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    text_pose.translation().z() = 1.0; // Define text_pose here
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();
  }

  void planAndVisualize()
  {
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 0.1;
    target_pose1.position.y = 0.2;
    target_pose1.position.z = 0.2;

    tf2::Quaternion orientation;
    orientation.setRPY(3.14 * 135 /180, 3.14 * 90 /180, 3.14 * 90 /180);
    target_pose1.orientation.w = orientation.getW();
    target_pose1.orientation.x = orientation.getX();
    target_pose1.orientation.y = orientation.getY();
    target_pose1.orientation.z = orientation.getZ();
    move_group_interface.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group_interface.getCurrentState()->getJointModelGroup("panda_arm"));
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  }

  ros::AsyncSpinner spinner;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_visual_tools::MoveItVisualTools visual_tools;
  Eigen::Isometry3d text_pose; // Declare text_pose here
};
*/