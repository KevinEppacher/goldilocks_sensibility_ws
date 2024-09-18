#include "Measurement_Panel.h"

MeasurementPanel::MeasurementPanel(ros::NodeHandle &nodehandler)
    : nh(nodehandler),                  // Initialize the NodeHandle
      tfBuffer(),                       // Initialize the buffer
      tfListener(tfBuffer),             // Correctly initialize the TransformListener
      selectedPoseIndex(-1)             // Initialize the selected pose index
{
    // Advertise the pose array publisher
    measurementPoseArrayPub = nh.advertise<geometry_msgs::PoseArray>("measurement_points", 1);
    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("measurement_points_names", 1);

    // Subscribe to the WrenchStamped topic
    wrenchSub = nh.subscribe("wrench", 1, &MeasurementPanel::wrenchCallback, this);
}

void MeasurementPanel::render()
{
    if (ImGui::Button("Add Current Pose"))
    {
        // Code um die aktuelle Roboterpose hinzuzufügen
        geometry_msgs::Pose currentPose;
        currentPose = getCurrentPose("base_link", "tool0_link");

        std::string poseName = "Pose " + std::to_string(poses.poses.size()); // Geben Sie der neuen Pose einen Namen
        addPose(currentPose, poseName);
        poses.header.frame_id = "base_link";

        measurementPoseArrayPub.publish(poses);
        updateMarkers(); // Aktualisieren der Marker nach dem Hinzufügen der Pose
    }

    ImGui::Text("Poses:");
    for (int i = 0; i < poses.poses.size(); ++i)
    {
        if (ImGui::Selectable((poseNames[i]).c_str(), selectedPoseIndex == i)) // Namen anzeigen
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
            currentPose = getCurrentPose("base_link", "tool0_link");
            updatePose(selectedPoseIndex, currentPose);
            poses.header.frame_id = "base_link";
            measurementPoseArrayPub.publish(poses);
            updateMarkers(); // Aktualisieren der Marker nach dem Aktualisieren der Pose
        }
    }
    measurementPoseArrayPub.publish(poses);

    plotWrenchData(latestWrench);
}

void MeasurementPanel::addPose(const geometry_msgs::Pose &pose, const std::string &poseName)
{
    poses.poses.push_back(pose);
    poseNames.push_back(poseName);
    updateMarkers(); // Aktualisieren der Marker nach dem Hinzufügen der Pose
}

void MeasurementPanel::updatePose(int index, const geometry_msgs::Pose &pose)
{
    if (index >= 0 && index < poses.poses.size())
    {
        poses.poses[index] = pose;
        updateMarkers(); // Aktualisieren der Marker nach dem Aktualisieren der Pose
    }
}

void MeasurementPanel::updateMarkers()
{
    visualization_msgs::MarkerArray markerArray;

    for (int i = 0; i < poses.poses.size(); ++i)
    {
        const auto& pose = poses.poses[i];

        // Erstellen Sie einen Marker für den Namen der Pose
        visualization_msgs::Marker textMarker;
        textMarker.header.frame_id = "base_link";
        textMarker.header.stamp = ros::Time::now();
        textMarker.ns = "pose_names";
        textMarker.id = i + poses.poses.size(); // Unterscheidung der IDs zwischen Pfeilen und Text
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.pose.position = pose.position; // Position des Textes auf der Pose
        textMarker.pose.position.z += 0.1; // Text etwas über der Pose anzeigen
        textMarker.scale.z = 0.05; // Schriftgröße
        textMarker.color.a = 1.0;  // Alpha
        textMarker.color.r = 1.0;  // Rot
        textMarker.color.g = 1.0;  // Grün
        textMarker.color.b = 1.0;  // Blau
        textMarker.text = poseNames[i]; // Name der Pose

        markerArray.markers.push_back(textMarker);
    }

    markerArrayPub.publish(markerArray);
}

geometry_msgs::Pose MeasurementPanel::getCurrentPose(const std::string &startFrame, const std::string &endFrame)
{
    geometry_msgs::Pose currentPose;

    ROS_INFO("Getting current pose from %s to %s", startFrame.c_str(), endFrame.c_str());

    // Check if startFrame or endFrame is empty
    if (startFrame.empty() || endFrame.empty()) {
        ROS_WARN("One or both of the frame IDs are empty: startFrame='%s', endFrame='%s'", startFrame.c_str(), endFrame.c_str());
        return currentPose; // Return an empty pose
    }

    try
    {
        // Wait for the transformation (timeout of 3 seconds)
        bool transform_available = tfBuffer.canTransform(startFrame, endFrame, ros::Time(0), ros::Duration(3.0));

        if (!transform_available)
        {
            ROS_WARN("Transform from %s to %s not available.", startFrame.c_str(), endFrame.c_str());
            return currentPose; // Return an empty pose in case of error
        }

        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(startFrame, endFrame, ros::Time(0));

        // If the transformation was successful
        currentPose.position.x = transformStamped.transform.translation.x;
        currentPose.position.y = transformStamped.transform.translation.y;
        currentPose.position.z = transformStamped.transform.translation.z;
        currentPose.orientation.x = transformStamped.transform.rotation.x;
        currentPose.orientation.y = transformStamped.transform.rotation.y;
        currentPose.orientation.z = transformStamped.transform.rotation.z;
        currentPose.orientation.w = transformStamped.transform.rotation.w;

        // Output transformation data
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
        // Optional: Return an empty pose or default pose in case of error
    }

    return currentPose;
}

void MeasurementPanel::plotWrenchData(const geometry_msgs::WrenchStamped &wrench)
{
    // Append the new data to the vectors
    forceX.push_back(wrench.wrench.force.x);
    forceY.push_back(wrench.wrench.force.y);
    forceZ.push_back(wrench.wrench.force.z);
    torqueX.push_back(wrench.wrench.torque.x);
    torqueY.push_back(wrench.wrench.torque.y);
    torqueZ.push_back(wrench.wrench.torque.z);

    // Ensure the vectors don't grow indefinitely
    if (forceX.size() > 100)
    {
        forceX.erase(forceX.begin());
        forceY.erase(forceY.begin());
        forceZ.erase(forceZ.begin());
        torqueX.erase(torqueX.begin());
        torqueY.erase(torqueY.begin());
        torqueZ.erase(torqueZ.begin());
    }

    // Convert double vectors to float vectors for ImGui
    std::vector<float> floatForceX(forceX.begin(), forceX.end());
    std::vector<float> floatForceY(forceY.begin(), forceY.end());
    std::vector<float> floatForceZ(forceZ.begin(), forceZ.end());
    std::vector<float> floatTorqueX(torqueX.begin(), torqueX.end());
    std::vector<float> floatTorqueY(torqueY.begin(), torqueY.end());
    std::vector<float> floatTorqueZ(torqueZ.begin(), torqueZ.end());

    // Plotting with ImGui
    // ImGui::Begin("Wrench Data Plot");

    // Force plot
    if (ImGui::CollapsingHeader("Force"))
    {
        ImGui::PlotLines("Force X", floatForceX.data(), floatForceX.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Force Y", floatForceY.data(), floatForceY.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Force Z", floatForceZ.data(), floatForceZ.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
    }

    // Torque plot
    if (ImGui::CollapsingHeader("Torque"))
    {
        ImGui::PlotLines("Torque X", floatTorqueX.data(), floatTorqueX.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Torque Y", floatTorqueY.data(), floatTorqueY.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Torque Z", floatTorqueZ.data(), floatTorqueZ.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
    }

    // ImGui::End();
}


