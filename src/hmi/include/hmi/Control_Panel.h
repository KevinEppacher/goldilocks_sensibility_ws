#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "imgui.h"
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/PoseArray.h>

class ControlPanel {
public:
    ControlPanel(ros::NodeHandle& N);
    void render();
private:
    void setupButtonCallback();
    void beginExternalControlButtonCallback();
    void defaultQUANTECCallback();
    void defaultUR10Callback();
    void moveRobotToHome(const std::string& planning_group, const std::string& target_pose);
    void poseCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void moveRobotToPose(const std::string& planning_group, const geometry_msgs::Pose& pose);
    geometry_msgs::Pose handlePose(int index, const geometry_msgs::PoseArray& poses);

    ros::NodeHandle nh;

    ros::Publisher indexControlURProgramPub;
    ros::Subscriber poseSub;
    geometry_msgs::PoseArray target_poses; // Store all received poses

    int setupValue = 1, externalControlValue = 2, defaultValue = 0;
};

#endif // CONTROL_PANEL_H
