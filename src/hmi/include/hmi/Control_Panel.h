#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "imgui.h"
#include <iostream>

class ControlPanel {
public:
    ControlPanel(ros::NodeHandle& nh);
    void render();
private:
    void setupButtonCallback();
    void beginExternalControlButtonCallback();
    void defaultButtonCallback();


    ros::Publisher indexControlURProgramPub;
    int setupValue = 1, externalControlValue = 2, defaultValue = 0;
};

#endif // CONTROL_PANEL_H
