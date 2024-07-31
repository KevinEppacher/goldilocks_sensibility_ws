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
    void beginMeasurementButtonCallback();

    ros::Publisher int_publisher_;
    int setup_value_;
    int measurement_value_;
};

#endif // CONTROL_PANEL_H
