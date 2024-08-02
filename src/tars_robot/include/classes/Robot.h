#ifndef ROBOT_H
#define ROBOT_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

namespace Robot {
    class ArticulatedRobot {
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
    };
}

#endif // ROBOT_H
