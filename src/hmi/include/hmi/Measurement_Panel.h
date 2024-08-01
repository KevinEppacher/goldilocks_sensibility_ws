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

class MeasurementPanel {
public:
    MeasurementPanel(ros::NodeHandle &nodehandler);
    void render();
    geometry_msgs::Pose getCurrentPose(const std::string& startFrame, const std::string& endFrame);

private:
    void addPose(const geometry_msgs::Pose &pose, const std::string &poseName);
    void updatePose(int index, const geometry_msgs::Pose &pose);
    void updateMarkers();

    ros::NodeHandle nh;
    ros::Publisher measurementPoseArrayPub;
    ros::Publisher markerArrayPub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::PoseArray poses;
    std::vector<std::string> poseNames;
    int selectedPoseIndex;
};

#endif // MEASUREMENT_PANEL_H
