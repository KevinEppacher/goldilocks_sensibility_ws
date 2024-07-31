#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

// Standard Libraries
#include "imgui.h"
#include <ros/ros.h>

// Custom Classes
#include "Control_Panel.h"
#include "Measurement_Panel.h"
#include "Plot_Panel.h"

class MainApplication
{
public:
    MainApplication(ros::NodeHandle &nh);
    void render();

private:
    ControlPanel controlPanel;
    MeasurementPanel measurementPanel;
    PlotPanel plotPanel;
    int currentTab;
};

#endif // MAIN_APPLICATION_H
