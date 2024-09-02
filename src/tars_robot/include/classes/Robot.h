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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace rvt = rviz_visual_tools;

namespace Robot {
    class ArticulatedRobot {
    public:
        ArticulatedRobot();
        void PTP(geometry_msgs::Pose target);
        void LIN(const std::string& reference_link, double distance, double linear_velocity);

    private:
        void planAndVisualize(geometry_msgs::Pose target);
        void logRobotState();
        void configureMoveGroup();

        std::string planningGroup;
        ros::AsyncSpinner spinner;
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // Separate visual tools for each reference frame
        moveit_visual_tools::MoveItVisualTools visual_tools_base;
        moveit_visual_tools::MoveItVisualTools visual_tools_table;
        Eigen::Isometry3d text_pose;
        std::string className = "ArticulatedRobot";
    };
}

#endif // ROBOT_H
