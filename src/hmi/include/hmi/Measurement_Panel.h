#ifndef MEASUREMENT_PANEL_H
#define MEASUREMENT_PANEL_H

#include "imgui.h"
#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

class MeasurementPanel {
public:
    MeasurementPanel();
    void render();
    void addPose(const geometry_msgs::Pose& pose);
    void updatePose(int index, const geometry_msgs::Pose& pose);
private:
    geometry_msgs::PoseArray poses;
    int selectedPoseIndex;
};

#endif // MEASUREMENT_PANEL_H
