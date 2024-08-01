#ifndef MEASUREMENT_PANEL_H
#define MEASUREMENT_PANEL_H

#include "imgui.h"
#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h> 
#include <visualization_msgs/Marker.h>  
#include <geometry_msgs/WrenchStamped.h>

class MeasurementPanel {
public:
    MeasurementPanel(ros::NodeHandle &nodehandler);
    void render();
    geometry_msgs::Pose getCurrentPose(const std::string& startFrame, const std::string& endFrame);

    //plot wrench data
    void plotWrenchData(const geometry_msgs::WrenchStamped &wrench);

private:
    void addPose(const geometry_msgs::Pose &pose, const std::string &poseName);
    void updatePose(int index, const geometry_msgs::Pose &pose);
    void updateMarkers();

    //get current wrench data
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        latestWrench = *msg;
    };

    ros::NodeHandle nh;
    ros::Publisher measurementPoseArrayPub;
    ros::Publisher markerArrayPub;
    //get wrench data
    ros::Subscriber wrenchSub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::PoseArray poses;
    std::vector<std::string> poseNames;
    int selectedPoseIndex;

    //plot wrench data
    std::vector<double> forceX;
    std::vector<double> forceY;
    std::vector<double> forceZ;
    std::vector<double> torqueX;
    std::vector<double> torqueY;
    std::vector<double> torqueZ;

    geometry_msgs::WrenchStamped latestWrench;
};

#endif // MEASUREMENT_PANEL_H 
