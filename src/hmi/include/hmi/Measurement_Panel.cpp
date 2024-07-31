#include "Measurement_Panel.h"

MeasurementPanel::MeasurementPanel() : selectedPoseIndex(-1)
{
    // Initialize any data members if needed
}

void MeasurementPanel::render()
{
    if (ImGui::Button("Add Current Pose")) {
        // Code to add the current robot pose
        geometry_msgs::Pose currentPose;
        // Assume getCurrentPose() is a function that gets the current pose of the robot
        // currentPose = getCurrentPose();
        addPose(currentPose);
    }

    ImGui::Text("Poses:");
    for (int i = 0; i < poses.poses.size(); ++i) {
        if (ImGui::Selectable(("Pose " + std::to_string(i)).c_str(), selectedPoseIndex == i)) {
            selectedPoseIndex = i;
        }
    }

    if (selectedPoseIndex != -1) {
        geometry_msgs::Pose& pose = poses.poses[selectedPoseIndex];

        // Temporary float variables to store converted double values
        float posX = static_cast<float>(pose.position.x);
        float posY = static_cast<float>(pose.position.y);
        float posZ = static_cast<float>(pose.position.z);
        float oriX = static_cast<float>(pose.orientation.x);
        float oriY = static_cast<float>(pose.orientation.y);
        float oriZ = static_cast<float>(pose.orientation.z);
        float oriW = static_cast<float>(pose.orientation.w);

        ImGui::InputFloat("X", &posX);
        ImGui::InputFloat("Y", &posY);
        ImGui::InputFloat("Z", &posZ);
        ImGui::InputFloat("Q1", &oriX);
        ImGui::InputFloat("Q2", &oriY);
        ImGui::InputFloat("Q3", &oriZ);
        ImGui::InputFloat("Q4", &oriW);

        // Update the pose with the new values
        pose.position.x = static_cast<double>(posX);
        pose.position.y = static_cast<double>(posY);
        pose.position.z = static_cast<double>(posZ);
        pose.orientation.x = static_cast<double>(oriX);
        pose.orientation.y = static_cast<double>(oriY);
        pose.orientation.z = static_cast<double>(oriZ);
        pose.orientation.w = static_cast<double>(oriW);

        if (ImGui::Button("Update Pose")) {
            updatePose(selectedPoseIndex, pose);
        }
    }
}

void MeasurementPanel::addPose(const geometry_msgs::Pose& pose)
{
    poses.poses.push_back(pose);
}

void MeasurementPanel::updatePose(int index, const geometry_msgs::Pose& pose)
{
    if (index >= 0 && index < poses.poses.size()) {
        poses.poses[index] = pose;
    }
}
