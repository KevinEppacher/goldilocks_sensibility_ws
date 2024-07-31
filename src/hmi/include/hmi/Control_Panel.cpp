#include "Control_Panel.h"

ControlPanel::ControlPanel(ros::NodeHandle& nh)
    : setup_value_(1), measurement_value_(2) {
    int_publisher_ = nh.advertise<std_msgs::Int32>("control_panel_topic", 10);
}

void ControlPanel::render() {
    // Set button size
    ImVec2 button_size(300, 100); // Width, Height

    // Set position and create Setup button
    ImGui::SetCursorPos(ImVec2(50, 50)); // X, Y
    if (ImGui::Button("Setup", button_size)) {
        setupButtonCallback();
    }

    // Set position and create Begin Measurement button
    ImGui::SetCursorPos(ImVec2(50, 200)); // X, Y
    if (ImGui::Button("Begin Measurement", button_size)) {
        beginMeasurementButtonCallback();
    }
}

void ControlPanel::setupButtonCallback() {
    std_msgs::Int32 msg;
    msg.data = setup_value_;
    int_publisher_.publish(msg);
    std::cout << "Setup button pressed, published value: " << setup_value_ << std::endl;
}

void ControlPanel::beginMeasurementButtonCallback() {
    std_msgs::Int32 msg;
    msg.data = measurement_value_;
    int_publisher_.publish(msg);
    std::cout << "Begin Measurement button pressed, published value: " << measurement_value_ << std::endl;
}
