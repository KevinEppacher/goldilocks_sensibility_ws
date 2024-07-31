#include "Control_Panel.h"

ControlPanel::ControlPanel(ros::NodeHandle &nh)
{
    indexControlURProgramPub = nh.advertise<std_msgs::Int32>("hmi/control_ur_program", 10);
}

void ControlPanel::render()
{
    // Set button size
    ImVec2 button_size(300, 100); // Width, Height

    // Set position and create Setup button
    ImGui::SetCursorPos(ImVec2(50, 50)); // X, Y
    if (ImGui::Button("Setup", button_size))
    {
        setupButtonCallback();
    }

    // Set position and create Begin Measurement button
    ImGui::SetCursorPos(ImVec2(50, 200)); // X, Y
    if (ImGui::Button("Begin External Control", button_size))
    {
        beginExternalControlButtonCallback();
    }

    // Set position and create Begin Measurement button
    ImGui::SetCursorPos(ImVec2(50, 350)); // X, Y
    if (ImGui::Button("Default Setting", button_size))
    {
        defaultButtonCallback();
    }
}

void ControlPanel::setupButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = setupValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Setup button pressed, published value: %d", setupValue);
}

void ControlPanel::beginExternalControlButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = externalControlValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Begin Measurement button pressed, published value: %d", externalControlValue);
}

void ControlPanel::defaultButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = defaultValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Default Setting button pressed, published value: %d", defaultValue);
}
