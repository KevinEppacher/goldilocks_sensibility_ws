#include "Measurement_Panel.h"

MeasurementPanel::MeasurementPanel()
    : tfListener(tfBuffer), selectedPoseIndex(-1) // Richtige Initialisierung hier
{
    // Initialisieren Sie hier Ihre Datenmitglieder falls nötig
}

void MeasurementPanel::render()
{
    if (ImGui::Button("Add Current Pose"))
    {
        // Code um die aktuelle Roboterpose hinzuzufügen
        geometry_msgs::Pose currentPose;
        currentPose = getCurrentPose("base", "tool0_controller");
        addPose(currentPose);
    }

    ImGui::Text("Poses:");
    for (int i = 0; i < poses.poses.size(); ++i)
    {
        if (ImGui::Selectable(("Pose " + std::to_string(i)).c_str(), selectedPoseIndex == i))
        {
            selectedPoseIndex = i;
        }
    }

    if (selectedPoseIndex != -1)
    {
        geometry_msgs::Pose &pose = poses.poses[selectedPoseIndex];

        // Temporäre float Variablen zum Speichern der konvertierten double Werte
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

        // Aktualisieren der Pose mit den neuen Werten
        pose.position.x = static_cast<double>(posX);
        pose.position.y = static_cast<double>(posY);
        pose.position.z = static_cast<double>(posZ);
        pose.orientation.x = static_cast<double>(oriX);
        pose.orientation.y = static_cast<double>(oriY);
        pose.orientation.z = static_cast<double>(oriZ);
        pose.orientation.w = static_cast<double>(oriW);

        if (ImGui::Button("Update Pose"))
        {
            geometry_msgs::Pose currentPose;
            currentPose = getCurrentPose("base", "tool0_controller");
            updatePose(selectedPoseIndex, currentPose);
        }
    }
}

void MeasurementPanel::addPose(const geometry_msgs::Pose &pose)
{
    poses.poses.push_back(pose);
}

void MeasurementPanel::updatePose(int index, const geometry_msgs::Pose &pose)
{
    if (index >= 0 && index < poses.poses.size())
    {
        poses.poses[index] = pose;
    }
}

geometry_msgs::Pose MeasurementPanel::getCurrentPose(const std::string &startFrame, const std::string &endFrame)
{
    geometry_msgs::Pose currentPose;

    ROS_INFO("Getting current pose from %s to %s", startFrame.c_str(), endFrame.c_str());

    try
    {
        // Warte auf die Transformation (Timeout von 3 Sekunden)
        bool transform_available = tfBuffer.canTransform(startFrame, endFrame, ros::Time(0), ros::Duration(3.0));

        if (!transform_available)
        {
            ROS_WARN("Transform from %s to %s not available.", startFrame.c_str(), endFrame.c_str());
            return currentPose; // Rückgabe einer leeren Pose im Fehlerfall
        }

        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(startFrame, endFrame, ros::Time(0));

        // Wenn die Transformation erfolgreich war
        currentPose.position.x = transformStamped.transform.translation.x;
        currentPose.position.y = transformStamped.transform.translation.y;
        currentPose.position.z = transformStamped.transform.translation.z;
        currentPose.orientation.x = transformStamped.transform.rotation.x;
        currentPose.orientation.y = transformStamped.transform.rotation.y;
        currentPose.orientation.z = transformStamped.transform.rotation.z;
        currentPose.orientation.w = transformStamped.transform.rotation.w;

        // Ausgabe der Transformationsdaten
        ROS_INFO("Translation: x=%f, y=%f, z=%f",
                 transformStamped.transform.translation.x,
                 transformStamped.transform.translation.y,
                 transformStamped.transform.translation.z);
        ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f",
                 transformStamped.transform.rotation.x,
                 transformStamped.transform.rotation.y,
                 transformStamped.transform.rotation.z,
                 transformStamped.transform.rotation.w);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not transform %s to %s: %s", startFrame.c_str(), endFrame.c_str(), ex.what());
        // Optional: Rückgabe einer leeren Pose oder standardmäßige Pose im Fehlerfall
    }

    return currentPose;
}