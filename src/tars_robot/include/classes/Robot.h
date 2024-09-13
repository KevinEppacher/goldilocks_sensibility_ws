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
#include <std_msgs/Bool.h>

namespace rvt = rviz_visual_tools;

namespace Robot {
    class ArticulatedRobot {
    public:
        ArticulatedRobot(ros::NodeHandle& nodeHandler);
        void PTP(geometry_msgs::Pose target, double velocityScaling, double accelerationScaling);
        void LIN(const std::string &referenceLink, double distance, double velocityScaling, double accelerationScaling, bool withActiveAirskin = false);
        geometry_msgs::Pose getCurrentPose(const std::string& referenceFrame = "tool0_link");

    private:
        void logRobotState();
        void configureMoveGroup();
        void loadParameters();
        void airskinStateCallback(const std_msgs::Bool::ConstPtr& msg);
        // void disableOctomapCollision();
        // void enableOctomapCollision();


        std::string planningGroup;
        ros::AsyncSpinner spinner;
        ros::NodeHandle nh;
        ros::Subscriber airskinStateSub;

        moveit::planning_interface::MoveGroupInterface moveGroup;
        moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
        moveit_visual_tools::MoveItVisualTools visualToolsBase;
        moveit_visual_tools::MoveItVisualTools visualToolsTable;
        Eigen::Isometry3d textPose;
        std::string className = "ArticulatedRobot";
        bool moveAllowed = true;

        // Parameters loaded from YAML
        double linearDistance;
        double acceptableFraction;
        std::string plannerId;
        int planningAttempts;
        double planningTime;
        double goalPositionTolerance;
        double goalOrientationTolerance;

    };
}

#endif // ROBOT_H